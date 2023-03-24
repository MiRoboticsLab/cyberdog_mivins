// Copyright (c) 2023-2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <mivins/common/camera.h>
#include <mivins/common/frame.h>
#include <mivins/common/point.h>
#include <mivins/common/container_helpers.h>
#include <mivins/direct/feature_patch_alignment.h>
#include <mivins/direct/feature_detector.h>
#include <mivins/direct/feature_detector_utilities.h>
#include <mivins/tracker/feature_tracker.h>
#include <mivins/tracker/feature_tracker_tools.h>

namespace mivins
{

    FeatureTracker::FeatureTracker(
        const FeatureTrackerOptions &options,
        const DetectorOptions &detector_options,
        const CameraBundlePtr &cams)
        : m_options(options), m_camera_size(cams->getNumCameras()), m_active_tracks(m_camera_size), m_inactive_tracks(m_camera_size)
    {
        for (size_t i = 0; i < cams->getNumCameras(); ++i)
        {
            m_feaure_detectors.push_back(
                feature_detector_utils::MakeDetector(
                    detector_options, cams->getCameraShared(i)));
        }
    }

    bool FeatureTracker::TrackFeaturesAndCheckDisparity(const FrameBundlePtr &frames)
    {
        TrackFrameBundle(frames);
        std::vector<size_t> num_tracked;
        std::vector<double> disparity;
        GetActiveTracksAndDisparityPerCam(m_options.init_disparity_pivot_ratio, &num_tracked, &disparity);

        size_t num_tracked_tot = std::accumulate(num_tracked.begin(), num_tracked.end(), 0u);
        double avg_disparity = std::accumulate(disparity.begin(), disparity.end(), 0.0) / disparity.size();
        std::cout << "Init: Tracked " << num_tracked_tot << " features with disparity = " << avg_disparity << "\n";

        if (num_tracked_tot < m_options.min_tracks_to_detect_new_features) //60
        {
            InitializeFeatureTrackers(frames);
            return false;
        }
        if (avg_disparity < m_options.init_disparity)
            return false;

        return true;
    }

    void FeatureTracker::TrackFeatures(const FrameBundlePtr &frames)
    {
        size_t n_tracked = TrackFrameBundle(frames);
        if (n_tracked < m_options.min_tracks_to_detect_new_features)
        {
            InitializeFeatureTrackers(frames);
        }
    }

    size_t FeatureTracker::TrackFrameBundle(const FrameBundlePtr &frames)
    {

        ClearInactiveTracks();
        size_t n = GetActiveTracksSize();
        if (n == 0)
            return 0;

        for (size_t camera_index = 0; camera_index < m_camera_size; ++camera_index)
        {
            FeatureTracksPerCam &tracks = m_active_tracks.at(camera_index);
            const FramePtr &cur_frame = frames->at(camera_index);
            std::vector<size_t> remove_indices;
            Keypoints new_keypoints(2, tracks.size());
            Scores new_scores(tracks.size());
            TrackIds new_track_ids(tracks.size());
            size_t new_keypoints_counter = 0;
            for (size_t track_index = 0; track_index < tracks.size(); ++track_index)
            {
                FeatureTrackPerId &track = tracks.at(track_index);
                const FeatureObs &ref_observation =
                    (m_options.klt_template_is_first_observation) ? track.front() : track.back();

                const ImgPyramid &ref_pyr = ref_observation.GetFrame()->img_pyr_;
                const ImgPyramid &cur_pyr = cur_frame->img_pyr_;

                // TODO(cfo): make work for feature coordinates with subpixel reference patch!
                // Currently not a problem because feature detector returns integer pos.
                Eigen::Vector2i ref_px_level_0 = ref_observation.GetPx().cast<int>();
                Keypoint cur_px_level_0 = track.back().GetPx();
                bool success = feature_patch_alignment::AlignPyr2D(
                    ref_pyr, cur_pyr,
                    m_options.klt_max_level, m_options.klt_min_level, m_options.klt_patch_sizes,
                    m_options.klt_max_iter, m_options.klt_min_update_squared,
                    ref_px_level_0, cur_px_level_0);
                if (success)
                {
                    new_keypoints.col(new_keypoints_counter) = cur_px_level_0;
                    new_scores(new_keypoints_counter) =
                        ref_observation.GetFrame()->score_vec_[ref_observation.GetFeatureIndex()];
                    new_track_ids(new_keypoints_counter) = track.GetTrackId();
                    track.pushBack(frames, camera_index, new_keypoints_counter);
                    ++new_keypoints_counter;
                }
                else
                {
                    remove_indices.push_back(track_index);
                    m_inactive_tracks.at(camera_index).push_back(track);
                }
            }

            auto new_tracks = mivins::common::container_helpers::eraseIndicesFromVector_DEPRECATED(tracks, remove_indices);
            tracks = new_tracks;

            // Insert new keypoints in frame.
            cur_frame->ResizeFeatureStorage(new_keypoints_counter);
            cur_frame->px_vec_ = new_keypoints.leftCols(new_keypoints_counter);
            cur_frame->score_vec_ = new_scores.head(new_keypoints_counter);
            cur_frame->track_id_vec_ = new_track_ids.head(new_keypoints_counter);
            cur_frame->num_features_ = new_keypoints_counter;

            frame_utils::ComputeNormalizedBearingVectors(
                cur_frame->px_vec_, *cur_frame->cam(), &cur_frame->f_vec_);

            std::cout << "Tracker: Frame-" << camera_index << " - tracked = " << new_keypoints_counter << std::endl;
        }

        return GetActiveTracksSize();
    }

    size_t FeatureTracker::TrackFeaturesAndAddPoints(const FrameBundlePtr &frames)
    {
        TrackFrameBundle(frames);
        DetectAndInitializeTracks(frames);

        return GetActiveTracksSize();
    }

    size_t FeatureTracker::DetectAndInitializeTracks(const FrameBundlePtr &frames)
    {
        CHECK_EQ(frames->size(), m_feaure_detectors.size());
        CHECK_EQ(frames->size(), m_active_tracks.size());

        for (size_t camera_index = 0; camera_index < m_camera_size; ++camera_index)
        {
            const FramePtr &frame = frames->at(camera_index);
            const size_t n_old = frame->num_features_;

            size_t max_n_features = std::min(m_options.init_features_size, int(m_feaure_detectors.at(camera_index)->grid_.size()));
            if (n_old < max_n_features)
            {
                const int n_features = max_n_features - n_old;
                Keypoints new_px;
                Scores new_scores;
                Levels new_levels;
                Gradients new_grads;
                FeatureTypes new_types;
                Bearings new_f;
                // Detect features
                m_feaure_detectors.at(camera_index)->ResetGrid();
                m_feaure_detectors.at(camera_index)->grid_.fillWithKeypoints(frame->px_vec_);
                m_feaure_detectors.at(camera_index)->Detect(frame->img_pyr_, frame->GetMask(), n_features, new_px, new_scores, new_levels, new_grads, new_types);

                std::vector<bool> success;
                frame->cam()->backProject3(new_px, &new_f, &success);

                new_f = new_f.array().rowwise() / new_f.colwise().norm().array();

                // Add features to frame.
                const size_t n_new = new_px.cols();
                frame->ResizeFeatureStorage(n_old + n_new);
                frame->px_vec_.middleCols(n_old, n_new) = new_px;
                frame->f_vec_.middleCols(n_old, n_new) = new_f;
                frame->grad_vec_.middleCols(n_old, n_new) = new_grads;
                frame->score_vec_.segment(n_old, n_new) = new_scores;
                frame->level_vec_.segment(n_old, n_new) = new_levels;
                frame->num_features_ = n_old + n_new;

                // Create a track for each feature
                FeatureTracksPerCam &tracks = m_active_tracks.at(camera_index);
                tracks.reserve(frame->NumFeatures());
                for (size_t feature_index = n_old; feature_index < frame->NumFeatures(); ++feature_index)
                {
                    const int new_track_id = GlobalPointId::GetNewPointId();
                    tracks.emplace_back(new_track_id);
                    tracks.back().pushBack(frames, camera_index, feature_index);
                    frame->track_id_vec_(feature_index) = new_track_id;
                }

                std::cout << "Tracker: Frame-" << camera_index << " - detected = " << n_new
                          << ", total = " << n_new + n_old << std::endl;
            }
        }

        return GetActiveTracksSize();
    }

    size_t FeatureTracker::InitializeFeatureTrackers(const FrameBundlePtr &frames)
    {
        if (m_options.reset_before_detection)
        {
            ClearActiveTracks();
            for (const FramePtr &frame : frames->frames_)
                frame->ClearFeatureStorage();
        }
        const size_t n = DetectAndInitializeTracks(frames);
        return n;
    }

    const FeatureTracksPerCam &FeatureTracker::GetActiveTracks(size_t camera_index) const
    {
        CHECK_LT(camera_index, m_active_tracks.size());
        return m_active_tracks.at(camera_index);
    }

    size_t FeatureTracker::GetActiveTracksSize() const
    {
        size_t total = 0;
        for (auto &tracks : m_active_tracks)
            total += tracks.size();
        return total;
    }

    void FeatureTracker::GetActiveTracksAndDisparityPerCam(
        double pivot_ratio,
        std::vector<size_t> *num_tracked,
        std::vector<double> *disparity) const
    {
        CHECK_NOTNULL(num_tracked);
        CHECK_NOTNULL(disparity);
        num_tracked->resize(m_camera_size);
        disparity->resize(m_camera_size);

        for (size_t i = 0; i < m_camera_size; ++i)
        {
            num_tracked->at(i) = m_active_tracks[i].size();
            disparity->at(i) = feature_tracker_tools::GetTracksDisparityPercentile(
                m_active_tracks[i], pivot_ratio);
        }
    }

    FrameBundlePtr FeatureTracker::GetOldestFrameInTrack(size_t camera_index) const
    {
        CHECK_LT(camera_index, m_active_tracks.size());
        const FeatureTrackPerId &track = m_active_tracks.at(camera_index).front();
        CHECK(!track.empty());
        return track.at(0).GetFrameBundle();
    }

    void FeatureTracker::ClearActiveTracks()
    {
        for (auto &track : m_active_tracks)
            track.clear();
    }

    void FeatureTracker::ClearInactiveTracks()
    {
        for (auto &track : m_inactive_tracks)
            track.clear();
    }

    void FeatureTracker::Clear()
    {
        ClearActiveTracks();
        ClearInactiveTracks();
        for (auto &detector : m_feaure_detectors)
            detector->ResetGrid();
    }

} // namespace mivins
