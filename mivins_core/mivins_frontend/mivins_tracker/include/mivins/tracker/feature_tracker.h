// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#pragma once

#include <numeric>
#include <mivins/common/types.h>
#include <mivins/common/camera_fwd.h>
#include <mivins/common/transformation.h>
#include <mivins/tracker/feature_tracker_obs.h>

namespace mivins
{

    class AbstractDetector;
    struct DetectorOptions;

    class FeatureTracker
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<FeatureTracker> Ptr;

        FeatureTracker() = delete;

        /// Provide number of frames per frame_bundle
        FeatureTracker(
            const FeatureTrackerOptions &options,
            const DetectorOptions &detector_options,
            const CameraBundlePtr &cams);

        bool TrackFeaturesAndCheckDisparity(const FrameBundlePtr &frames);

        void TrackFeatures(const FrameBundlePtr &frames);

        /// Tracks a frame bundle. Returns number of tracked features.
        size_t TrackFrameBundle(const FrameBundlePtr &frames);

        size_t TrackFeaturesAndAddPoints(const FrameBundlePtr &frames);

        /// Extract new features
        size_t DetectAndInitializeTracks(const FrameBundlePtr &frames);

        size_t InitializeFeatureTrackers(const FrameBundlePtr &frames);

        const FeatureTracksPerCam &GetActiveTracks(size_t camera_index) const;

        size_t GetActiveTracksSize() const;

        /// pivot_ration needs to be in range(0,1) and if 0.5 the disparity that
        /// is returned per frame is the median. If 0.25 it means that 25% of the
        /// tracks have a higher disparity than returned.
        void GetActiveTracksAndDisparityPerCam(
            double pivot_ratio,
            std::vector<size_t> *num_tracked,
            std::vector<double> *disparity) const;

        FrameBundlePtr GetOldestFrameInTrack(size_t camera_index) const;

        void ClearActiveTracks();

        void ClearInactiveTracks();

        void Clear();

        //private:
        FeatureTrackerOptions m_options;

        // A vector for each frame in the bundle.
        const size_t m_camera_size;
        std::vector<std::shared_ptr<AbstractDetector>> m_feaure_detectors;
        std::vector<FeatureTracksPerCam,
                    Eigen::aligned_allocator<FeatureTracksPerCam>>
            m_active_tracks;
        std::vector<FeatureTracksPerCam,
                    Eigen::aligned_allocator<FeatureTracksPerCam>>
            m_inactive_tracks;
    };

} // namespace mivins
