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

#include <numeric>
#include <mivins/direct/patch_matcher.h>
#include <mivins/common/point.h>
#include <mivins/common/frame.h>
#include <mivins/stereo_triangulation.h>
#include <mivins/direct/feature_detector.h>
#include <mivins/tracker/feature_tracker.h>

namespace mivins
{

    StereoTriangulation::StereoTriangulation(
        const StereoTriangulationOptions &options,
        const AbstractDetector::Ptr &feature_detector)
        : options_(options), feature_detector_(feature_detector)
    {
        ;
    }

    void StereoTriangulation::compute(const FramePtr &frame0,
                                      const FramePtr &frame1)
    {
        // Check if there is something to do
        if (frame0->NumLandmarks() >= options_.triangulate_n_features)
        {
            VLOG(5) << "Calling stereo triangulation with sufficient number of features"
                    << " has no effect.";
            return;
        }

        // Detect new features.
        Keypoints new_px;
        Levels new_levels;
        Scores new_scores;
        Gradients new_grads;
        FeatureTypes new_types;
        const size_t max_n_features = feature_detector_->grid_.size();
        feature_detector_->Detect(
            frame0->img_pyr_, frame0->GetMask(), max_n_features, new_px,
            new_scores, new_levels, new_grads, new_types);
        if (new_px.cols() == 0)
        {
            LOG_ERROR_STREAM("Stereo Triangulation: No features detected.");
            return;
        }

        // Compute and normalize all bearing vectors.
        Bearings new_f;
        frame_utils::ComputeNormalizedBearingVectors(new_px, *frame0->cam(), &new_f);

        // Add features to first frame.
        const long n_old = static_cast<long>(frame0->NumFeatures());
        const long n_new = new_px.cols();
        frame0->ResizeFeatureStorage(
            frame0->num_features_ + static_cast<size_t>(n_new));
        frame0->px_vec_.middleCols(n_old, n_new) = new_px;
        frame0->f_vec_.middleCols(n_old, n_new) = new_f;
        frame0->grad_vec_.middleCols(n_old, n_new) = new_grads;
        frame0->score_vec_.segment(n_old, n_new) = new_scores;
        frame0->level_vec_.segment(n_old, n_new) = new_levels;
        frame0->num_features_ += static_cast<size_t>(n_new);
        frame0->type_vec_.insert(
            frame0->type_vec_.begin() + n_old, new_types.cbegin(), new_types.cend());

        // We only want a limited number of features. Therefore, we create a random
        // vector of indices that we will process.
        std::vector<size_t> indices(static_cast<size_t>(n_new));
        std::iota(indices.begin(), indices.end(), n_old);
        long n_corners = std::count_if(
            new_types.begin(), new_types.end(),
            [](const FeatureType &t)
            { return t == FeatureType::kCorner; });

        // shuffle twice before we prefer corners!
        std::random_shuffle(indices.begin(), indices.begin() + n_corners);
        std::random_shuffle(indices.begin() + n_corners, indices.end());

        // now for all maximum corners, initialize a new seed
        size_t n_succeded = 0, n_failed = 0;
        const size_t n_desired =
            options_.triangulate_n_features - frame0->NumLandmarks();
        //note: we checked already at start that n_desired will be larger than 0

        // reserve space for features in second frame
        if (frame1->num_features_ + n_desired > frame1->landmark_vec_.size())
        {
            frame1->ResizeFeatureStorage(frame1->num_features_ + n_desired);
        }

        PatchMatcher matcher;
        matcher.m_patch_matcher_options.max_epi_search_steps = 500;
        matcher.m_patch_matcher_options.subpix_refinement = true;
        const Transformation T_f1f0 = frame1->T_cam_body_ * frame0->T_body_cam_;
        for (const size_t &i_ref : indices)
        {
            matcher.m_patch_matcher_options.align_1d = isEdgelet(frame0->type_vec_[i_ref]); // TODO(cfo): check effect
            FloatType depth = 0.0;
            FeatureWrapper ref_ftr = frame0->GetFeatureWrapper(i_ref);
            PatchMatcher::MatchResult res =
                matcher.FindEpipolarMatchDirect(
                    *frame0, *frame1, T_f1f0, ref_ftr, options_.mean_depth_inv,
                    options_.min_depth_inv, options_.max_depth_inv, depth);

            if (res == PatchMatcher::MatchResult::kSuccess)
            {
                const Position xyz_world = frame0->T_world_cam() * (frame0->f_vec_.col(static_cast<int>(i_ref)) * depth);
                PointPtr new_point(new Point(xyz_world));
                frame0->landmark_vec_[i_ref] = new_point;
                frame0->track_id_vec_(static_cast<int>(i_ref)) = new_point->Id();
                new_point->AddObservation(frame0, i_ref);

                const int i_cur = static_cast<int>(frame1->num_features_);
                frame1->type_vec_[static_cast<size_t>(i_cur)] = ref_ftr.type;
                frame1->level_vec_[i_cur] = ref_ftr.level;
                frame1->px_vec_.col(i_cur) = matcher.m_px_cur;
                frame1->f_vec_.col(i_cur) = matcher.m_f_cur;
                frame1->score_vec_[i_cur] = ref_ftr.score;
                GradientVector g = matcher.m_affine_cur_ref * ref_ftr.grad;
                frame1->grad_vec_.col(i_cur) = g.normalized();
                frame1->landmark_vec_[static_cast<size_t>(i_cur)] = new_point;
                frame1->track_id_vec_(i_cur) = new_point->Id();
                new_point->AddObservation(frame1, static_cast<size_t>(i_cur));
                frame1->num_features_++;
                ++n_succeded;
            }
            else
            {
                ++n_failed;
            }
            if (n_succeded >= n_desired)
                break;
        }
        VLOG(20) << "Stereo: Triangulated " << n_succeded << " features,"
                 << n_failed << " failed.";
    }

} // namespace mivins
