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

#include <mivins/channel_frame_stereo.h>
#include <mivins/frontend_local_map.h>
#include <mivins/common/frame.h>
#include <mivins/common/point.h>
#include <mivins/pose_optimizer.h>
#include <mivins/bundle_adjustment.h>
#include <mivins/img_align/sparse_img_align.h>
#include <mivins/direct/depth_optimization.h>
#include <mivins/stereo_triangulation.h>
#include <mivins/direct/feature_detector.h>
#include <mivins/direct/feature_detector_utilities.h>
#include <mivins/reprojector.h>
#include <mivins/initialization.h>
#include <mivins/utils/performance_monitor.h>

namespace mivins
{

    ChannelFrameStereo::ChannelFrameStereo(
        const BaseOptions &base_options,
        const DepthOptimizationOptions &depth_filter_options,
        const DetectorOptions &feature_detector_options,
        const InitializationOptions &init_options,
        const StereoTriangulationOptions &stereo_options,
        const ReprojectorOptions &reprojector_options,
        const FeatureTrackerOptions &tracker_options,
        const CameraBundle::Ptr &stereo_camera)
        : ChannelFrameBase(
              base_options, reprojector_options, depth_filter_options,
              feature_detector_options, init_options, tracker_options, stereo_camera)
    {
        // init initializer
        DetectorOptions detector_options = feature_detector_options;
        detector_options.detector_type = DetectorType::kFast;
        stereo_triangulation_.reset(
            new StereoTriangulation(
                stereo_options,
                feature_detector_utils::MakeDetector(
                    detector_options, cams_->getCameraShared(0))));
    }
    int ChannelFrameStereo::GetType()
    {
        return 1;
    }
    UpdateResult ChannelFrameStereo::ProcessFrameBundle()
    {
        UpdateResult res = UpdateResult::kFailure;
        if (stage_ == Stage::kTracking)
            res = ProcessFrame();
        else if (stage_ == Stage::kInitializing)
        {
            TryToInitialize();

            if (!first_img_) // && vins_backend_->tryToInitialize(new_frames_))
            {
                if (options_.backend_opt)
                {
                    if (vins_backend_->tryToInitialize(new_frames_))
                    {
                        SVO_INFO_STREAM("Init finish...");
                        stage_ = Stage::kTracking;
                        tracking_quality_ = TrackingQuality::kGood;
                        res = UpdateResult::kKeyframe;
                    }
                    else
                    {
                        res = UpdateResult::kDefault;
                    }
                }
                else
                {
                    SVO_INFO_STREAM("Init finish...");
                    stage_ = Stage::kTracking;
                    tracking_quality_ = TrackingQuality::kGood;
                    res = UpdateResult::kKeyframe;
                }
            }
            else
            {
                res = UpdateResult::kDefault;
            }
        }

        return res;
    }
    void ChannelFrameStereo::TryToInitialize()
    {
        const FramePtr &frame0 = new_frames_->at(0);
        const FramePtr &frame1 = new_frames_->at(1);
        if (first_img_)
        {
            SVO_DEBUG_STREAM("T_W_B="<<new_frames_->Get_T_W_B());

            stereo_triangulation_->compute(frame0, frame1);
            
            
            size_t init_lm_cnt = 0;
            for(int i = 0; i < (int)frame0->NumLandmarks(); ++i)
            {
              const PointPtr point = frame0->landmark_vec_[i];
              if(point == nullptr) continue;

              Eigen::Vector3d pt_world = point->pos3d_in_w;
              Eigen::Vector3d pt_cam = frame0->T_cam_world() * pt_world;
              if(pt_cam.z() > 0.25f)
                ++init_lm_cnt;
            }
            
            if (frame0->NumLandmarks() < options_.init_min_fts)
            {
                printf("First frame: %ld landmarks is constructed, is less than the required number: %ld\n", 
                    init_lm_cnt, options_.init_min_fts);
                //SVO_DEBUG_STREAM("First frame: "<<init_lm_cnt<<" landmarks is constructed, is less than the required number:"<<options_.init_min_fts);    
                return ;
            }
            else
            {
                //printf("First frame: %ld landmarks is constructed\n", frame0->NumLandmarks());
                SVO_DEBUG_STREAM("First frame:"<<frame0->NumLandmarks()<<"landmarks is constructed.");
                first_img_ = false;
                frame_utils::GetSceneDepth(frame0, depth_median_, depth_min_, depth_max_);
                depth_filter_->AddKeyframe(frame0, depth_median_, 0.5 * depth_min_, depth_median_ * 1.5);
                depth_filter_->AddKeyframe(frame1, depth_median_, 0.5 * depth_min_, depth_median_ * 1.5);
            }
        }
        else
        {
            size_t n_tracked_features = 0;
            SparseImageAlignment();
            n_tracked_features = ProjectMapInFrame();

            n_tracked_features = OptimizePose();
            OptimizeStructure(new_frames_, options_.structure_optimization_max_pts, 5);

            SetDetectorOccupiedCells(0, stereo_triangulation_->feature_detector_);
            SetDetectorOccupiedCells(1, depth_filter_->m_feature_detector);

            UpgradeSeedsToFeatures(frame0);
            UpgradeSeedsToFeatures(frame1);

            stereo_triangulation_->compute(frame0, frame1);

            depth_filter_->AddKeyframe(frame0, depth_median_, 0.5 * depth_min_, depth_median_ * 1.5);
            depth_filter_->AddKeyframe(frame1, depth_median_, 0.5 * depth_min_, depth_median_ * 1.5);

            depth_filter_->SeedsUpdate(overlap_kfs_.at(0), frame0);
            depth_filter_->SeedsUpdate(overlap_kfs_.at(1), frame1);

            if (options_.update_seeds_with_old_keyframes)
            {
                const FramePtr &last_frame0 = last_frames_->at(0);
                const FramePtr &last_frame1 = last_frames_->at(1);

                depth_filter_->SeedsUpdate({frame0}, last_frame0);
                depth_filter_->SeedsUpdate({frame0}, last_frame1);
                depth_filter_->SeedsUpdate({frame1}, last_frame0);
                depth_filter_->SeedsUpdate({frame1}, last_frame1);
                for (const FramePtr &old_keyframe : overlap_kfs_.at(0))
                {
                    depth_filter_->SeedsUpdate({frame0}, old_keyframe);
                    depth_filter_->SeedsUpdate({frame1}, old_keyframe);
                }
            }
        }

        frame0->SetKeyframe();
        frame1->SetKeyframe();
        map_->AddKeyframe(frame0, true);
        map_->AddKeyframe(frame1, true);
        return;
    }

    void ChannelFrameStereo::AddImages(
        const cv::Mat &img_left,
        const cv::Mat &img_right,
        const uint64_t timestamp)
    {
        // TODO: deprecated
        AddImageBundle({img_left, img_right}, timestamp);
    }

    //receive vins intialize result && set
    UpdateResult ChannelFrameStereo::ProcessVinsFrame()
    {
        SVO_INFO_STREAM("Init: Selected first frame.");
        stage_ = Stage::kTracking;
        tracking_quality_ = TrackingQuality::kGood;
        return UpdateResult::kKeyframe;
    }

    UpdateResult ChannelFrameStereo::JointInitialize()
    {
        return UpdateResult::kDefault;
    }
    UpdateResult ChannelFrameStereo::ProcessFrame()
    {
        // ---------------------------------------------------------------------------
        // tracking

        //save original pose
        Transformation T_W_B = new_frames_->Get_T_W_B();

        // STEP 1: Sparse Image Align
        size_t n_tracked_features = 0;

        // TicToc t_image_align;
        SparseImageAlignment();
        if (options_.save_time_consumption)
        {
            savet.precision(9);
            // savet<<t_image_align.toc()<<" ";
        }

        // STEP 2: FrontendLocalMap Reprojection & Feature Align
        // TicToc t_project;
        n_tracked_features = ProjectMapInFrame();
        if (options_.save_time_consumption)
        {
            savet.precision(9);
            // savet<<t_project.toc()<<" ";
        }
        //std::cout << "s2:n=" << n_tracked_features << "," << options_.quality_min_fts << std::endl;
        SVO_DEBUG_STREAM("s2:n=" << n_tracked_features << "," << options_.quality_min_fts);

        std::vector<int> index;
        int n_count = 0;
        {
            {

                const mivins::FramePtr &f0 = new_frames_->at(0);
                for (size_t j = 0; j < f0->num_features_; ++j)
                {
                    if (f0->landmark_vec_[j] && f0->landmark_vec_[j]->obs_.size() > 3)
                    {
                        index.push_back(f0->landmark_vec_[j]->id_); //frame->landmark_vec_[i]->id_
                        n_count++;
                    }
                }
            }
            {
                const mivins::FramePtr &f1 = new_frames_->at(1);
                for (size_t j = 0; j < f1->num_features_; ++j)
                {
                    if (f1->landmark_vec_[j] && f1->landmark_vec_[j]->obs_.size() > 3)
                    {
                        auto it = find(index.begin(), index.end(), f1->landmark_vec_[j]->id_);
                        if (it == index.end())
                            n_count++;
                    }
                }
            }
        }

        //std::cout << "n_count=" << n_count << std::endl;
        SVO_DEBUG_STREAM("n_count=" << n_count);
        if(n_tracked_features < 10)
        {
            ++bad_reproj_cnt_;
            if(bad_reproj_cnt_ >= 3)
                bad_reproj_ = true;
            if(bad_reproj_)
            {
                is_abnormal_ = true;
                bad_reproj_ = false;
                bad_reproj_cnt_ = 0;
            }
            new_frames_->Set_T_W_B(T_W_B);
            return MakeKeyframe();
        }
        bad_reproj_cnt_ = 0;

        // if(n_count < options_.quality_min_fts)
        // {
        //   return MakeKeyframe(); // force stereo triangulation to recover
        // }
        if (n_tracked_features < options_.quality_min_fts)
        {
            return MakeKeyframe(); // force stereo triangulation to recover
        }

        // STEP 3: Pose & Structure Optimization
        // TicToc t_opti;
        if(options_.light_optimize)
        {
            n_tracked_features = OptimizePose();
            //std::cout << "s3:n" << n_tracked_features << std::endl;
            SVO_DEBUG_STREAM("s3:n" << n_tracked_features);

            if(n_tracked_features < 10)
            {
                ++bad_optimize_cnt_;
                if(bad_optimize_cnt_ >= 3)
                    bad_optimize_ = true;
                if(bad_optimize_)
                {
                    is_abnormal_ = true;
                    bad_optimize_ = false;
                    bad_optimize_cnt_ = 0;
                }
                new_frames_->Set_T_W_B(T_W_B);
                return MakeKeyframe();
            }
            bad_optimize_cnt_ = 0;

            if (n_tracked_features < options_.quality_min_fts)
            {
                return MakeKeyframe(); // force stereo triangulation to recover
            }
            OptimizeStructure(new_frames_, options_.structure_optimization_max_pts, 5);
        }
        if (options_.save_time_consumption)
        {
            savet.precision(9);
            // savet<<t_opti.toc()<<" ";
        }

        //if(n_count<70)
        //  return MakeKeyframe();
        // return if tracking bad
        SetTrackingQuality(n_tracked_features);
        if (tracking_quality_ == TrackingQuality::kInsufficient)
        {
            return MakeKeyframe(); // force stereo triangulation to recover
        }

        // ---------------------------------------------------------------------------
        // select keyframe
        frame_utils::GetSceneDepth(new_frames_->at(0), depth_median_, depth_min_, depth_max_);
        if (!need_new_kf_(new_frames_->at(0)->T_f_w_))
        {
            for (size_t i = 0; i < new_frames_->size(); ++i)
                depth_filter_->SeedsUpdate(overlap_kfs_.at(i), new_frames_->at(i));
            return UpdateResult::kDefault;
        }
        SVO_DEBUG_STREAM("New keyframe selected.");
        return MakeKeyframe();
    }

    UpdateResult ChannelFrameStereo::MakeKeyframe()
    {
        static size_t kf_counter = 0;
        const size_t kf_id = kf_counter++ % cams_->numCameras();
        const size_t other_id = kf_counter % cams_->numCameras();
        CHECK(kf_id != other_id);

        // ---------------------------------------------------------------------------
        // add extra features when num tracked is critically low!
        if (1) //new_frames_->NumLandmarks() < options_.kfselect_numkfs_lower_thresh)
        {
            SetDetectorOccupiedCells(0, stereo_triangulation_->feature_detector_);
            new_frames_->at(other_id)->SetKeyframe();
            map_->AddKeyframe(new_frames_->at(other_id), true);

            UpgradeSeedsToFeatures(new_frames_->at(other_id));
            stereo_triangulation_->compute(new_frames_->at(0), new_frames_->at(1));
        }

        // ---------------------------------------------------------------------------
        // new keyframe selected
        new_frames_->at(kf_id)->SetKeyframe();
        map_->AddKeyframe(new_frames_->at(kf_id), true);

        UpgradeSeedsToFeatures(new_frames_->at(kf_id));

        // init new depth-filters, set feature-detection grid-cells occupied that
        // already have a feature
        {
            DepthOptimization::ULockT lock(depth_filter_->m_feature_detector_mut);
            SetDetectorOccupiedCells(kf_id, depth_filter_->m_feature_detector);
        } // release lock
        depth_filter_->AddKeyframe(
            new_frames_->at(kf_id), depth_median_, 0.5 * depth_min_, depth_median_ * 1.5);
        depth_filter_->AddKeyframe(
            new_frames_->at(other_id), depth_median_, 0.5 * depth_min_, depth_median_ * 1.5);
        depth_filter_->SeedsUpdate(overlap_kfs_.at(0), new_frames_->at(0));
        depth_filter_->SeedsUpdate(overlap_kfs_.at(1), new_frames_->at(1));

        // TEST
        // {
        if (options_.update_seeds_with_old_keyframes)
        {
            depth_filter_->SeedsUpdate({new_frames_->at(0)}, last_frames_->at(0));
            depth_filter_->SeedsUpdate({new_frames_->at(0)}, last_frames_->at(1));
            depth_filter_->SeedsUpdate({new_frames_->at(1)}, last_frames_->at(0));
            depth_filter_->SeedsUpdate({new_frames_->at(1)}, last_frames_->at(1));
            for (const FramePtr &old_keyframe : overlap_kfs_.at(0))
            {
                depth_filter_->SeedsUpdate({new_frames_->at(0)}, old_keyframe);
                depth_filter_->SeedsUpdate({new_frames_->at(1)}, old_keyframe);
            }
        }
        // }

        // if limited number of keyframes, remove the one furthest apart
        while (map_->size() > options_.max_n_kfs && options_.max_n_kfs > 2)
        {
            if (true)
            {
                // deal differently with map for ceres backend
                map_->RemoveOldestKeyframe();
                map_->RemoveOldestKeyframe();
            }
            else
            {
                FramePtr furthest_frame =
                    map_->GetFurthestKeyframe(new_frames_->at(kf_id)->GetCameraPosInWorld());
                map_->RemoveKeyframe(furthest_frame->GetFrameId());
            }
        }
        return UpdateResult::kKeyframe;
    }

    void ChannelFrameStereo::ResetAll()
    {
        backend_scale_initialized_ = true;
        ResetVisionFrontendCommon();
    }

} // namespace mivins
