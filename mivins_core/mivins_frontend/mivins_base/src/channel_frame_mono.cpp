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

#include <mivins/channel_frame_mono.h>
#include <mivins/frontend_local_map.h>
#include <mivins/common/frame.h>
#include <mivins/common/point.h>
#include <mivins/img_align/sparse_img_align.h>
#include <mivins/bundle_adjustment.h>
#include <mivins/direct/depth_optimization.h>
#include <mivins/tracker/feature_tracker_obs.h>
#include <mivins/initialization.h>
#include <mivins/direct/feature_detector.h>
#include <mivins/direct/feature_detector_utilities.h>
#include <mivins/reprojector.h>
#include <mivins/utils/performance_monitor.h>

namespace mivins
{

    ChannelFrameMono::ChannelFrameMono(
        const BaseOptions &base_options,
        const DepthOptimizationOptions &depth_filter_options,
        const DetectorOptions &feature_detector_options,
        const InitializationOptions &init_options,
        const ReprojectorOptions &reprojector_options,
        const FeatureTrackerOptions &tracker_options,
        const CameraBundle::Ptr &cam)
        : ChannelFrameBase(
              base_options, reprojector_options, depth_filter_options,
              feature_detector_options, init_options, tracker_options, cam)
    {
        ;
    }

    UpdateResult ChannelFrameMono::ProcessFrameBundle()
    {
        UpdateResult res = UpdateResult::kFailure;
        if (stage_ == Stage::kTracking)
        {
            res = ProcessFrame();
        }
        else if (stage_ == Stage::kInitializing)
        {
            res = ProcessFirstFrame();
            // if(estimator->svoInitializeFrameMono(new_frames_))
            // {
            //   res= ProcessVinsFrameMono();
            // }
            // else{
            //   res = UpdateResult::kDefault;
            // }
        }
        else if (stage_ == Stage::kRelocalization)
        {
            res = RelocalizeFrame(Transformation(), reloc_keyframe_);
        }
        return res;
    }
    int ChannelFrameMono::GetType()
    {
        return 0;
    }
    void ChannelFrameMono::AddImage(
        const cv::Mat &img,
        const uint64_t timestamp)
    {
        AddImageBundle({img}, timestamp);
    }
    //receive vins intialize result && set
    UpdateResult ChannelFrameMono::ProcessVinsFrameMono()
    {
        // make old frame keyframe
        /*
  FrameBundlePtr& ref_framebundle = estimator->getRefFrameBundle();
  ref_framebundle->SetKeyframe();
  ref_framebundle->at(0)->SetKeyframe();
  if(bundle_adjustment_type_==BundleAdjustmentType::kCeres)
  {
     map_->AddKeyframe(ref_framebundle->at(0),
                       bundle_adjustment_type_==BundleAdjustmentType::kCeres);
  }
  else if(bundle_adjustment_type_==BundleAdjustmentType::kGtsam)
  {
    CHECK(bundle_adjustment_)
        << "bundle_adjustment_type_ is kGtsam but bundle_adjustment_ is NULL";
    bundle_adjustment_->bundleAdjustment(ref_framebundle);
  }
  else
  {
    map_->AddKeyframe(ref_framebundle->at(0), false);
  }
  // make new frame keyframe
  NewFrame()->SetKeyframe();
  frame_utils::GetSceneDepth(NewFrame(), depth_median_, depth_min_, depth_max_);
  VLOG(40) << "Current Frame Depth: " << "min: " << depth_min_
          << ", max: " << depth_max_ << ", median: " << depth_median_;
  depth_filter_->AddKeyframe(
              NewFrame(), depth_median_, 0.5*depth_min_, depth_median_*1.5);
  VLOG(40) << "Updating seeds in second frame using last frame...";
  depth_filter_->SeedsUpdate({ NewFrame() }, LastFrameUnsafe());

  // add frame to map

  map_->AddKeyframe(NewFrame(),
                    bundle_adjustment_type_==BundleAdjustmentType::kCeres);
  SVO_INFO_STREAM("Init: Selected first frame.");
  stage_ = Stage::kTracking;
  tracking_quality_ = TrackingQuality::kGood;
  */
        return UpdateResult::kKeyframe;
    }

    UpdateResult ChannelFrameMono::ProcessFirstFrame()
    {
        if (!initializer_->have_depth_prior_)
        {
            initializer_->setDepthPrior(options_.init_map_scale);
        }
        if (have_rotation_prior_)
        {
            VLOG(2) << "Setting absolute orientation prior";
            initializer_->setAbsoluteOrientationPrior(
                NewFrame()->T_cam_imu().GetRotation() * R_imu_world_);
        }
        const auto res = initializer_->AddFrameBundle(new_frames_);

        if (res == InitResult::kTracking)
            return UpdateResult::kDefault;

        // make old frame keyframe
        initializer_->frames_ref_->SetKeyframe();
        initializer_->frames_ref_->at(0)->SetKeyframe();
        if (bundle_adjustment_type_ == BundleAdjustmentType::kCeres)
        {
            map_->AddKeyframe(initializer_->frames_ref_->at(0),
                              bundle_adjustment_type_ == BundleAdjustmentType::kCeres);
        }
        // else if(bundle_adjustment_type_==BundleAdjustmentType::kGtsam)
        // {
        //   CHECK(bundle_adjustment_)
        //       << "bundle_adjustment_type_ is kGtsam but bundle_adjustment_ is NULL";
        //   bundle_adjustment_->bundleAdjustment(initializer_->frames_ref_);
        // }
        else
        {
            map_->AddKeyframe(initializer_->frames_ref_->at(0), false);
        }
        // make new frame keyframe
        NewFrame()->SetKeyframe();
        frame_utils::GetSceneDepth(NewFrame(), depth_median_, depth_min_, depth_max_);
        VLOG(40) << "Current Frame Depth: "
                 << "min: " << depth_min_
                 << ", max: " << depth_max_ << ", median: " << depth_median_;
        depth_filter_->AddKeyframe(
            NewFrame(), depth_median_, 0.5 * depth_min_, depth_median_ * 1.5);
        VLOG(40) << "Updating seeds in second frame using last frame...";
        depth_filter_->SeedsUpdate({NewFrame()}, LastFrameUnsafe());

        // add frame to map

        map_->AddKeyframe(NewFrame(),
                          bundle_adjustment_type_ == BundleAdjustmentType::kCeres);
        stage_ = Stage::kTracking;
        tracking_quality_ = TrackingQuality::kGood;
        initializer_->reset();
        VLOG(1) << "Init: Selected second frame, triangulated initial map.";
        return UpdateResult::kKeyframe;
    }
    UpdateResult ChannelFrameMono::JointInitialize()
    {
        return UpdateResult::kDefault;
    }
    UpdateResult ChannelFrameMono::ProcessFrame()
    {
        VLOG(40) << "Updating seeds in overlapping keyframes...";
        // this is useful when the pipeline is with the backend,
        // where we should have more accurate pose at this moment
        depth_filter_->SeedsUpdate(overlap_kfs_.at(0), LastFrame());

        // ---------------------------------------------------------------------------
        // tracking

        // STEP 1: Sparse Image Align
        VLOG(40) << "===== Sparse Image Alignment =====";
        size_t n_total_observations = 0;
        SparseImageAlignment();

        // STEP 2: FrontendLocalMap Reprojection & Feature Align
        VLOG(40) << "===== Project FrontendLocalMap to Current Frame =====";
        n_total_observations = ProjectMapInFrame();
        if (n_total_observations < options_.quality_min_fts)
        {
            LOG(WARNING) << "Not enough feature after reprojection: "
                         << n_total_observations;
            return UpdateResult::kFailure;
        }

        // STEP 3: Pose & Structure Optimization
        // redundant when using ceres backend
        if (bundle_adjustment_type_ != BundleAdjustmentType::kCeres)
        {
            VLOG(40) << "===== Pose Optimization =====";
            n_total_observations = OptimizePose();
            if (n_total_observations < options_.quality_min_fts)
            {
                LOG(WARNING) << "Not enough feature after pose optimization."
                             << n_total_observations;
                return UpdateResult::kFailure;
            }
            OptimizeStructure(new_frames_, options_.structure_optimization_max_pts, 5);
        }

        // return if tracking bad
        SetTrackingQuality(n_total_observations);
        if (tracking_quality_ == TrackingQuality::kInsufficient)
            return UpdateResult::kFailure;

        // ---------------------------------------------------------------------------
        // select keyframe
        VLOG(40) << "===== Keyframe Selection =====";
        frame_utils::GetSceneDepth(NewFrame(), depth_median_, depth_min_, depth_max_);
        VLOG(40) << "Current Frame Depth: "
                 << "min: " << depth_min_
                 << ", max: " << depth_max_ << ", median: " << depth_median_;
        initializer_->setDepthPrior(depth_median_);
        initializer_->have_depth_prior_ = true;
        if (!need_new_kf_(NewFrame()->T_f_w_) || tracking_quality_ == TrackingQuality::kBad || stage_ == Stage::kRelocalization)
        {
            if (tracking_quality_ == TrackingQuality::kGood)
            {
                VLOG(40) << "Updating seeds in overlapping keyframes...";
                CHECK(!overlap_kfs_.empty());
                // now the seed is updated at the beginning of next frame
                //      depth_filter_->SeedsUpdate(overlap_kfs_.at(0), NewFrame());
            }
            return UpdateResult::kDefault;
        }
        NewFrame()->SetKeyframe();
        VLOG(40) << "New keyframe selected.";

        // ---------------------------------------------------------------------------
        // new keyframe selected
        if (depth_filter_->m_opt_options.extra_map_points)
        {
            depth_filter_->m_sec_feature_detector->ResetGrid();
            OccupandyGrid2D map_point_grid(depth_filter_->m_sec_feature_detector->grid_);
            reprojector_utils::ReprojectMapPoints(
                NewFrame(), overlap_kfs_.at(0),
                reprojectors_.at(0)->options_, &map_point_grid);
            DepthOptimization::ULockT lock(depth_filter_->m_feature_detector_mut);
            feature_detector_utils::MergeGrids(
                map_point_grid, &depth_filter_->m_sec_feature_detector->grid_);
        }
        UpgradeSeedsToFeatures(NewFrame());
        // init new depth-filters, set feature-detection grid-cells occupied that
        // already have a feature
        //
        // TODO: we should also project all seeds first! to make sure that we don't
        //       initialize seeds in the same location!!! this can be done in the
        //       depth-filter
        //
        {
            DepthOptimization::ULockT lock(depth_filter_->m_feature_detector_mut);
            SetDetectorOccupiedCells(0, depth_filter_->m_feature_detector);

        } // release lock
        depth_filter_->AddKeyframe(
            NewFrame(), depth_median_, 0.5 * depth_min_, depth_median_ * 1.5);

        if (options_.update_seeds_with_old_keyframes)
        {
            VLOG(40) << "Updating seeds in current frame using last frame...";
            depth_filter_->SeedsUpdate({NewFrame()}, LastFrameUnsafe());
            VLOG(40) << "Updating seeds in current frame using overlapping keyframes...";
            for (const FramePtr &old_keyframe : overlap_kfs_.at(0))
                depth_filter_->SeedsUpdate({NewFrame()}, old_keyframe);
        }

        VLOG(40) << "Updating seeds in overlapping keyframes...";
        //  depth_filter_->SeedsUpdate(overlap_kfs_.at(0), NewFrame());

        // add keyframe to map
        map_->AddKeyframe(NewFrame(),
                          bundle_adjustment_type_ == BundleAdjustmentType::kCeres);

        // if limited number of keyframes, remove the one furthest apart
        if (options_.max_n_kfs > 2)
        {
            while (map_->size() > options_.max_n_kfs)
            {
                if (bundle_adjustment_type_ == BundleAdjustmentType::kCeres)
                {
                    // deal differently with map for ceres backend
                    map_->RemoveOldestKeyframe();
                }
                else
                {
                    FramePtr furthest_frame = map_->GetFurthestKeyframe(NewFrame()->GetCameraPosInWorld());
                    map_->RemoveKeyframe(furthest_frame->GetFrameId());
                }
            }
        }
        return UpdateResult::kKeyframe;
    }

    UpdateResult ChannelFrameMono::RelocalizeFrame(
        const Transformation & /*T_cur_ref*/,
        const FramePtr &ref_keyframe)
    {
        ++relocalization_n_trials_;
        if (ref_keyframe == nullptr)
            return UpdateResult::kFailure;

        VLOG_EVERY_N(1, 20) << "Relocalizing frame";
        FrameBundle::Ptr ref_frame(new FrameBundle({ref_keyframe}, ref_keyframe->bundleId()));
        last_frames_ = ref_frame;
        UpdateResult res = ProcessFrame();
        if (res == UpdateResult::kDefault)
        {
            // Reset to default mode.
            stage_ = Stage::kTracking;
            relocalization_n_trials_ = 0;
            VLOG(1) << "Relocalization successful.";
        }
        else
        {
            // reset to last well localized pose
            NewFrame()->T_f_w_ = ref_keyframe->T_f_w_;
        }
        return res;
    }

    void ChannelFrameMono::ResetAll()
    {
        if (bundle_adjustment_type_ == BundleAdjustmentType::kCeres)
        {
            // with the ceres backend we have to make sure to initialize the scale
            backend_scale_initialized_ = false;
        }
        else
        {
            backend_scale_initialized_ = true;
        }
        ResetVisionFrontendCommon();
    }

    FramePtr ChannelFrameMono::LastFrame() const
    {
        return (last_frames_ == nullptr) ? nullptr : last_frames_->at(0);
    }

    const FramePtr &ChannelFrameMono::NewFrame() const
    {
        return new_frames_->frames_[0];
    }

    const FramePtr &ChannelFrameMono::LastFrameUnsafe() const
    {
        return last_frames_->frames_[0];
    }

    bool ChannelFrameMono::HaveLastFrame() const
    {
        return (last_frames_ != nullptr);
    }

} // namespace mivins
