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

#include <mivins/channel_frame_array.h>
#include <mivins/frontend_local_map.h>
#include <mivins/common/frame.h>
#include <mivins/common/point.h>
#include <mivins/pose_optimizer.h>
#include <mivins/bundle_adjustment.h>
#include <mivins/img_align/sparse_img_align.h>
#include <mivins/direct/depth_optimization.h>
#include <mivins/stereo_triangulation.h>
#include <mivins/direct/feature_detector.h>
#include <mivins/reprojector.h>
#include <mivins/initialization.h>
#include <mivins/utils/performance_monitor.h>

namespace mivins
{

    ChannelFrameArray::ChannelFrameArray(
        const BaseOptions &base_options,
        const DepthOptimizationOptions &depth_filter_options,
        const DetectorOptions &feature_detector_options,
        const InitializationOptions &init_options,
        const ReprojectorOptions &reprojector_options,
        const FeatureTrackerOptions &tracker_options,
        const CameraBundle::Ptr &cameras)
        : ChannelFrameBase(
              base_options, reprojector_options, depth_filter_options,
              feature_detector_options, init_options, tracker_options, cameras)
    {
        ;
    }

    UpdateResult ChannelFrameArray::ProcessFrameBundle()
    {
        UpdateResult res = UpdateResult::kFailure;
        if (stage_ == Stage::kTracking)
            res = ProcessFrame();
        else if (stage_ == Stage::kInitializing)
            res = ProcessSecondFrame();
        else if (stage_ == Stage::kInitializing)
            res = ProcessFirstFrame();

        return res;
    }

    void ChannelFrameArray::AddImages(
        const std::vector<cv::Mat> &images,
        const uint64_t timestamp)
    {
        // TODO: deprecated
        AddImageBundle(images, timestamp);
    }
    int ChannelFrameArray::GetType()
    {
        return 2;
    }
    UpdateResult ChannelFrameArray::ProcessFirstFrame()
    {
        // Add first frame to initializer. It may return a failure if not enough features
        // can be detected, i.e., we are in a texture-less area.
        initializer_->setDepthPrior(options_.init_map_scale);
        if (initializer_->AddFrameBundle(new_frames_) == InitResult::kFailure)
            return UpdateResult::kDefault;

        stage_ = Stage::kTracking;
        LOG_INFO_STREAM("Init: Selected first frame.");
        return UpdateResult::kKeyframe;
    }

    UpdateResult ChannelFrameArray::ProcessSecondFrame()
    {
        vk::Timer t;

        initializer_->setDepthPrior(options_.init_map_scale);
        auto res = initializer_->AddFrameBundle(new_frames_);
        LOG_INFO_STREAM("Init: Processing took " << t.stop() * 1000 << "ms");

        if (res == InitResult::kFailure)
            return UpdateResult::kFailure;
        else if (res == InitResult::kNoKeyframe)
            return UpdateResult::kDefault;

        // make old frame keyframe
        for (const FramePtr &frame : initializer_->frames_ref_->frames_)
        {
            frame->SetKeyframe();
            map_->AddKeyframe(frame,
                              bundle_adjustment_type_ == BundleAdjustmentType::kCeres);
        }

        // make new frame keyframe
        for (const FramePtr &frame : new_frames_->frames_)
        {
            frame->SetKeyframe();
            frame_utils::GetSceneDepth(frame, depth_median_, depth_min_, depth_max_);
            depth_filter_->AddKeyframe(
                frame, depth_median_, 0.5 * depth_min_, depth_median_ * 1.5);
            map_->AddKeyframe(frame,
                              bundle_adjustment_type_ == BundleAdjustmentType::kCeres);
        }

        stage_ = Stage::kTracking;
        initializer_->reset();
        LOG_INFO_STREAM("Init: Selected second frame, triangulated initial map.");
        return UpdateResult::kKeyframe;
    }

    UpdateResult ChannelFrameArray::ProcessFrame()
    {
        // ---------------------------------------------------------------------------
        // tracking

        // STEP 1: Sparse Image Align
        size_t n_tracked_features = 0;
        SparseImageAlignment();

        // STEP 2: FrontendLocalMap Reprojection & Feature Align
        n_tracked_features = ProjectMapInFrame();
        if (n_tracked_features < options_.quality_min_fts)
            return UpdateResult::kFailure;

        // STEP 3: Pose & Structure Optimization
        n_tracked_features = OptimizePose();
        if (n_tracked_features < options_.quality_min_fts)
            return UpdateResult::kFailure;
        OptimizeStructure(new_frames_, options_.structure_optimization_max_pts, 5);

        // return if tracking bad
        SetTrackingQuality(n_tracked_features);
        if (tracking_quality_ == TrackingQuality::kInsufficient)
            return UpdateResult::kFailure;

        // ---------------------------------------------------------------------------
        // select keyframe
        frame_utils::GetSceneDepth(new_frames_->at(0), depth_median_, depth_min_, depth_max_);
        //if(!need_new_kf_(new_frames_->at(0)->T_f_w_))
        if (frame_counter_ % 4 != 0)
        {
            for (size_t i = 0; i < new_frames_->size(); ++i)
                depth_filter_->SeedsUpdate(overlap_kfs_.at(i), new_frames_->at(i));
            return UpdateResult::kDefault;
        }
        LOG_DEBUG_STREAM("New keyframe selected.");

        for (size_t i = 0; i < new_frames_->size(); ++i)
            MakeKeyframe(i);

        return UpdateResult::kKeyframe;
    }

    UpdateResult ChannelFrameArray::MakeKeyframe(const size_t camera_id)
    {
        const FramePtr &frame = new_frames_->at(camera_id);

        // ---------------------------------------------------------------------------
        // new keyframe selected
        frame->SetKeyframe();
        map_->AddKeyframe(frame,
                          bundle_adjustment_type_ == BundleAdjustmentType::kCeres);
        UpgradeSeedsToFeatures(frame);

        // init new depth-filters, set feature-detection grid-cells occupied that
        // already have a feature
        {
            DepthOptimization::ULockT lock(depth_filter_->m_feature_detector_mut);
            SetDetectorOccupiedCells(camera_id, depth_filter_->m_feature_detector);
        } // release lock
        double depth_median = -1, depth_min, depth_max;
        if (!frame_utils::GetSceneDepth(frame, depth_median, depth_min, depth_max))
        {
            depth_min = 0.2;
            depth_median = 3.0;
            depth_max = 100;
        }
        LOG_DEBUG_STREAM("Average Depth " << frame->cam()->getLabel() << ": " << depth_median);
        depth_filter_->AddKeyframe(
            new_frames_->at(camera_id), depth_median, 0.5 * depth_min, depth_median * 1.5);
        depth_filter_->SeedsUpdate(overlap_kfs_.at(camera_id), frame);

        // if limited number of keyframes, remove the one furthest apart
        while (map_->size() > options_.max_n_kfs && options_.max_n_kfs > 2)
        {
            if (bundle_adjustment_type_ == BundleAdjustmentType::kCeres)
            {
                // deal differently with map for ceres backend
                map_->RemoveOldestKeyframe();
            }
            else
            {
                FramePtr furthest_frame = map_->GetFurthestKeyframe(frame->GetCameraPosInWorld());
                map_->RemoveKeyframe(furthest_frame->GetFrameId());
            }
        }
        return UpdateResult::kKeyframe;
    }

    void ChannelFrameArray::ResetAll()
    {
        backend_scale_initialized_ = true;
        ResetVisionFrontendCommon();
        depth_filter_->Reset();
    }

} // namespace mivins
