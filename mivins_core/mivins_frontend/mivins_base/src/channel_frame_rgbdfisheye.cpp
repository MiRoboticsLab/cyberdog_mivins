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

#include <mivins/channel_frame_rgbdfisheye.h>
#include <mivins/frontend_local_map.h>
#include <mivins/common/frame.h>
#include <mivins/common/point.h>
#include <mivins/img_align/sparse_img_align.h>
#include <mivins/bundle_adjustment.h>
#include <mivins/direct/depth_optimization.h>
#include <mivins/initialization.h>
#include <mivins/direct/feature_detector.h>
#include <mivins/direct/feature_detector_utilities.h>
#include <mivins/reprojector.h>
#include <mivins/utils/performance_monitor.h>

#include <mivins/tracker/feature_tracker_tools.h>
#include <mivins/tracker/feature_tracker_obs.h>
#include <mivins/direct/feature_patch_alignment.h>

//#include <opencv2/highgui/highgui_c.h>
//#include <opencv2/highgui/highgui.hpp>

namespace mivins
{

    ChannelFrameRgbdFisheye::ChannelFrameRgbdFisheye(
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
        depth_cam_id_ = 0;

        init_landmark_size_ = 20; // 5 for sim with imu

        init_disparity_ = 10.0f;

        reproj_error_thresh_ = 2.0f;

        tracker_options_ = tracker_options;

        feature_detector_ = feature_detector_utils::MakeDetector(
            feature_detector_options, cam->getCameraShared(depth_cam_id_));

        DetectorOptions detector_options_init = feature_detector_options;
        detector_options_init.detector_type = DetectorType::kFast;

        tracker_.reset(new FeatureTracker(tracker_options, detector_options_init, cam));
        for (size_t i = 0; i < cam->getNumCameras(); ++i)
        {
            depth_img_min_ = cam->getCameraShared(i)->getDepthMin();
            depth_img_max_ = cam->getCameraShared(i)->getDepthMax();
            depth_img_scale_ = cam->getCameraShared(i)->getDepthScale();
            if (depth_img_min_ > 0.0f && depth_img_max_ > 0.0f && depth_img_scale_ > 0.0f)
            {
                std::cout << " RgbdFisheye depth min: " << depth_img_min_ << "; "
                          << " RgbdFisheye depth max: " << depth_img_max_ << "; "
                          << " RgbdFisheye depth scale: " << depth_img_scale_ << "\n";
                break;
            }
        }
    }

    UpdateResult ChannelFrameRgbdFisheye::ProcessFrameBundle()
    {
        UpdateResult res = UpdateResult::kFailure;
        if (stage_ == Stage::kTracking)
        {
            res = ProcessFrame();
        }
        else if (stage_ == Stage::kInitializing)
        {
            if (!options_.use_joint_init) //use_backend_init 使用后端初始化，前端跟踪特征点
            {
                tracker_->TrackFeaturesAndAddPoints(new_frames_);
                /*
      {
        for(const FramePtr& frame : new_frames_->frames_)
        {
          cv::Mat img_8u = frame->img();
          cv::Mat img_rgb(img_8u.size(), CV_8UC3);
          cv::cvtColor(img_8u, img_rgb, cv::COLOR_GRAY2RGB);
          for (size_t feature_index = 0; feature_index < frame->num_features_; ++feature_index)
          {
            cv::circle(
                img_rgb,
                cv::Point2f(frame->px_vec_.col(static_cast<int>(feature_index))[0], frame->px_vec_.col(static_cast<int>(feature_index))[1]),
                3,
                cv::Scalar(0,255,0), -1);

            std::string text = std::to_string(frame->track_id_vec_(feature_index));
            int font_face = cv::FONT_HERSHEY_SIMPLEX; 
            double font_scale = 0.4;
            int thickness = 1;
            int baseline;
            //获取文本框的长宽
            cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
          
            //将文本框居中绘制
            cv::Point origin; 
            origin.x = frame->px_vec_.col(static_cast<int>(feature_index))[0] - text_size.width / 2;
            origin.y = frame->px_vec_.col(static_cast<int>(feature_index))[1] + text_size.height / 2;
            cv::putText(img_rgb, text, origin, font_face, font_scale, cv::Scalar(0, 0, 255), thickness, 8, 0);
            // std::cout << "frame" << frame->id_ << " : feature[" << feature_index << "] = " << frame->px_vec_.col(static_cast<int>(feature_index))[0] <<
            // " , " << frame->px_vec_.col(static_cast<int>(feature_index))[1] << std::endl;
          }
          
          ostringstream ossl;
          string baseNamel = "/home/mi/catkin_ws_vinsvo_xiaomi_ros2/save_image/";
          ossl << baseNamel << new_frames_->GetBundleId()<<"_"<< frame->GetNFrameIndex() << ".png";
          imwrite(ossl.str().c_str(), img_rgb);
        }
      }
      */
                bool flag = vins_backend_->tryToInitialize(new_frames_);
                if (!flag)
                    res = UpdateResult::kDefault;
                else
                {
                    res = InitFrontend();
                }
            }
            else if (options_.use_joint_init) //使用前端两帧初始化，后续前端跟踪，填充后端滑窗，后端优化后初始化成功
            {
                std::cout << "use_joint_init....." << std::endl;
                res = JointInitialize();
            }
        }
        else if (stage_ == Stage::kRelocalization)
        {
            static int relocn = 0;
            relocn++;
            std::cout << "IN[ChannelFrameRgbdFisheye::ProcessFrameBundle]---------------- --------wrong --------------- --------kreloc-------- --------!" << relocn << std::endl;
            //res = RelocalizeFrame(Transformation(), reloc_keyframe_);   //TODO fisheye
            bool bUseRlocSvo = false;
            if (bUseRlocSvo)
            {
                res = RelocalizeFramebundle(Transformation(), reloc_keyframe_bundle_);
            }
        }
        return res;
    }

    UpdateResult ChannelFrameRgbdFisheye::InitFrontend()
    {
        if (options_.backend_opt && vins_backend_ && vins_backend_->isEstimatorValid())
        {
            int last_KF_landmark_cnt = 0;
            std::cout << "window_size: " << window_size_ << std::endl;
            mivins::FrameBundlePtr frame_bundle = vins_backend_->getFrameBundle(window_size_);
            for (size_t k = 0; k < frame_bundle->at(0)->num_features_; ++k)
            {
                if (frame_bundle->at(0)->landmark_vec_[k])
                    ++last_KF_landmark_cnt;
            }
            if (last_KF_landmark_cnt < (int)options_.quality_min_fts * 1.5)
            {
                std::cout << "initialize failed\n";
                vins_backend_->reset();
                ResetAll();
                return UpdateResult::kDefault;
            }

            double prv_ts = 0.0f;
            for (int i = 0; i <= window_size_; ++i)
            {
                mivins::FrameBundlePtr frame_bundle = vins_backend_->getFrameBundle(i);
                if (!frame_bundle || frame_bundle->GetMinTimestampSeconds() == prv_ts)
                    continue;

                prv_ts = frame_bundle->GetMinTimestampSeconds();

                frame_bundle->SetKeyframe();
                for (size_t j = 0; j < frame_bundle->size(); ++j)
                {
                    frame_bundle->at(j)->SetKeyframe();
                    map_->AddKeyframe(frame_bundle->at(j), true);
                }
                last_frames_ = frame_bundle;
            }

            stage_ = Stage::kTracking;
            tracking_quality_ = TrackingQuality::kGood;
            return UpdateResult::kKeyframe;
        }
    }
    int ChannelFrameRgbdFisheye::GetType()
    {
        return 3;
    }
    void ChannelFrameRgbdFisheye::AddImage(
        const cv::Mat &img,
        const uint64_t timestamp)
    {
        AddImageBundle({img}, timestamp);
    }

    // UpdateResult ChannelFrameRgbdFisheye::ProcessFirstFrame()
    // {
    //   const auto res = initializer_->AddFrameBundle(new_frames_);
    //   for(const FramePtr& frame : new_frames_->frames_)
    //   {
    //     map_->AddKeyframe(frame,
    //                       bundle_adjustment_type_==BundleAdjustmentType::kCeres);
    //   }
    //   while(map_->size() > options_.max_n_kfs && options_.max_n_kfs > 3)
    //   {
    //     if(true)//bundle_adjustment_type_==BundleAdjustmentType::kCeres)
    //     {
    //       // deal differently with map for ceres backend
    //       map_->RemoveOldestKeyframe();
    //       map_->RemoveOldestKeyframe();
    //       map_->RemoveOldestKeyframe();
    //     }
    //   }
    //   return UpdateResult::kDefault;

    // }

    UpdateResult ChannelFrameRgbdFisheye::JointInitialize()
    {
        if (!init_)
        {
            bool res = false;
            res = AddInitFrameBundle(new_frames_);
            std::cout << "jointInit:" << std::endl;

            if (res)
            {
                cv::Size img_size = new_frames_->at(depth_cam_id_)->img_pyr_[0].size();
                cv::Mat mask = cv::Mat(img_size, CV_8UC1, cv::Scalar(255));
                new_frames_->at(depth_cam_id_)->cam_->setMask(mask);

                const FrameBundlePtr frames_ref = tracker_->GetOldestFrameInTrack(0);
                vins_backend_->tryToInitialize(frames_ref);

                vins_backend_->tryToInitialize(new_frames_);

                frames_ref->SetKeyframe();
                for (size_t i = 0; i < frames_ref->size(); ++i)
                {
                    frames_ref->at(i)->SetKeyframe();
                    map_->AddKeyframe(frames_ref->at(i), true);
                }

                new_frames_->SetKeyframe();
                for (size_t i = 0; i < new_frames_->size(); ++i)
                {
                    new_frames_->at(i)->SetKeyframe();
                    map_->AddKeyframe(frames_ref->at(i), true);

                    frame_utils::GetSceneDepth(new_frames_->at(i), depth_median_, depth_min_, depth_max_);
                    depth_filter_->AddKeyframe(
                        new_frames_->at(i), depth_median_, 0.5 * depth_min_, depth_median_ * 1.5);
                }

                init_ = true;
            }

            return UpdateResult::kDefault;
        }
        else
        {
            GetMotionPriorWithDepth();

            size_t n_tracked_features = 0;
            SparseImageAlignment();

            n_tracked_features = ProjectMapInFrame();

            n_tracked_features = OptimizePose();
            // if(n_total_observations < options_.quality_min_fts)
            // {
            // }
            OptimizeStructure(new_frames_, options_.structure_optimization_max_pts, 5);

            if (!need_new_kf_(new_frames_->at(0)->T_f_w_))
            {
                for (size_t i = 0; i < new_frames_->size(); ++i)
                    depth_filter_->SeedsUpdate(overlap_kfs_.at(i), new_frames_->at(i));
                return UpdateResult::kDefault;
            }

            for (size_t i = 0; i < new_frames_->size(); ++i)
                MakeKeyframe(i);

            bool res = vins_backend_->tryToInitialize(new_frames_);
            if (res)
            {
                if (options_.backend_opt && vins_backend_ && vins_backend_->isEstimatorValid())
                {
                    double prv_ts = 0.0f;
                    for (int i = 0; i <= window_size_; ++i)
                    {
                        // auto &frame_bundle = vins_backend_->vins_estimator_->FrameBundles_[i];
                        mivins::FrameBundlePtr frame_bundle = vins_backend_->getFrameBundle(i);
                        if (!frame_bundle || frame_bundle->GetMinTimestampSeconds() == prv_ts)
                            continue;

                        prv_ts = frame_bundle->GetMinTimestampSeconds();

                        int landmark_cnt = 0;
                        for (size_t k = 0; k < frame_bundle->at(0)->num_features_; ++k)
                        {
                            if (frame_bundle->at(0)->landmark_vec_[k])
                                ++landmark_cnt;
                        }
                        std::cout << "landmark_cnt: " << landmark_cnt << std::endl;
                    }
                }

                SVO_INFO_STREAM("Init done.");
                stage_ = Stage::kTracking;
                tracking_quality_ = TrackingQuality::kGood;
                return UpdateResult::kKeyframe;
            }
            else
                return UpdateResult::kDefault;
        }
    }

    bool ChannelFrameRgbdFisheye::AddInitFrameBundle(const mivins::FrameBundlePtr &frames_cur)
    {
        cv::Mat depth_img = frames_cur->at(depth_cam_id_)->depth_image_.clone();
        cv::Size img_size = frames_cur->at(depth_cam_id_)->img_pyr_[0].size();
        cv::Mat mask = cv::Mat(img_size, CV_8UC1, cv::Scalar(255));
        std::cout << "depth_img_max_=" << depth_img_max_ << std::endl;
        if (!depth_img.empty())
        {
            cv::threshold(depth_img, mask, depth_img_max_ * depth_img_scale_,
                          255, cv::THRESH_BINARY_INV);
            mask.convertTo(mask, CV_8UC1);
        }
        frames_cur->at(depth_cam_id_)->cam_->setMask(mask);

        bool res = false;

        //res = trackFeaturesAndCheckDisparity(frames_cur);
        res = tracker_->TrackFeaturesAndCheckDisparity(frames_cur);

        // just for test
        std::unordered_map<size_t, size_t> matches_cur_ref;
        if (last_frames_)
        {
            //getFeatureMatches(frames_cur->at(0), last_frames_->at(0), matches_cur_ref);
            //drawFeatureMatches(frames_cur->at(0), last_frames_->at(0), matches_cur_ref);
            feature_tracker_tools::GetFeatureMatches(frames_cur->at(0), last_frames_->at(0), matches_cur_ref);
            feature_tracker_tools::DrawFeatureMatches(frames_cur->at(0), last_frames_->at(0), matches_cur_ref);
        }

        if (!res)
            return false;

        int depth_cam_id = -1;
        for (size_t i = 0; i < frames_cur->size(); ++i)
        {
            if (!frames_cur->at(i)->depth_image_.empty())
            {
                depth_cam_id = i;
                break;
            }
        }
        CHECK_NE(depth_cam_id, -1);

        const FrameBundlePtr frames_ref = tracker_->GetOldestFrameInTrack(0);

        const FramePtr &frame_ref = frames_ref->at(depth_cam_id);
        const FramePtr &frame_cur = frames_cur->at(depth_cam_id);

        Transformation T_cur_ref;
        res = GetRelPoseWithPnPRansac(frame_cur, frame_ref, T_cur_ref);
        // if(!res || T_cur_ref.GetPosition().norm() < 0.1f)
        //   return false;
        if (!res)
        {
            //trackNewFeatures(frames_cur);
            tracker_->InitializeFeatureTrackers(frames_cur);
            Transformation tmp_pose;
            if (imu_handler_)
            {
                std::cout << "have_init_rotation_prior_..." << std::endl;
                // Vector3d t(0,0,0);
                // tmp_pose = Transformation(R_init_imu_world_.Inverse(), t);
                // frames_cur->Set_T_W_B(tmp_pose);
                // const FramePtr &frame_cur = frames_cur->at(depth_cam_id);
                // std::cout<<"out:"<<frame_cur->T_imu_world().getRotationMatrix()<<std::endl;
                SetInitialPose(frames_cur);
            }
            else
            {
                tmp_pose.setIdentity();
                frames_cur->Set_T_W_B(tmp_pose);
            }
            return false;
        }
        Transformation cur_T_b_w = frame_ref->T_body_cam_ * T_cur_ref * frame_ref->T_f_w_;
        Transformation cur_T_w_b = cur_T_b_w.Inverse();
        frames_cur->Set_T_W_B(cur_T_w_b);

        res = TriangulateAndInitializePoints(frames_cur, frames_ref);
        if (!res)
        {
            //trackNewFeatures(frames_cur);
            tracker_->InitializeFeatureTrackers(frames_cur);
            Transformation tmp_pose;
            if (imu_handler_)
            {
                std::cout << "have_init_rotation_prior_..." << std::endl;
                // Vector3d t(0,0,0);
                // tmp_pose = Transformation(R_init_imu_world_.Inverse(), t);
                // frames_cur->Set_T_W_B(tmp_pose);
                // const FramePtr &frame_cur = frames_cur->at(depth_cam_id);
                // std::cout<<"out:"<<frame_cur->T_imu_world().getRotationMatrix()<<std::endl;
                SetInitialPose(frames_cur);
            }
            else
            {
                tmp_pose.setIdentity();
                frames_cur->Set_T_W_B(tmp_pose);
            }
            return false;
        }
        return true;
    }

    bool ChannelFrameRgbdFisheye::GetRelPoseWithPnPRansac(
        const FramePtr &frame_cur, const FramePtr &frame_ref,
        Transformation &T_cur_ref)
    {
        const int init_3d_points_size = 25;
        std::unordered_map<size_t, size_t> matches_cur_ref;
        //getFeatureMatches(frame_cur, frame_ref, matches_cur_ref);
        feature_tracker_tools::GetFeatureMatches(frame_cur, frame_ref, matches_cur_ref);

        if (matches_cur_ref.size() < 8)
            return false;
        RejectWithF(frame_cur, frame_ref, matches_cur_ref);

        // Case 1: get T_cur_ref with 3D points(ref frame) & 2D points(cur frame)
        std::vector<cv::Point3f> ref_matched_3d;
        std::vector<cv::Point2f> cur_matched_norm;

        for (auto &it : matches_cur_ref)
        {
            size_t idx_cur = it.first;
            size_t idx_ref = it.second;

            Eigen::Vector2d px_ref = frame_ref->px_vec_.col(idx_ref);
            int x_ref = round(px_ref.x()), y_ref = round(px_ref.y());
            float depth_ref = frame_ref->GetValidDepthFromImage(y_ref, x_ref);

            if (depth_ref < depth_img_min_ || depth_ref > depth_img_max_)
                continue;

            Eigen::Vector3d f_cur = frame_cur->f_vec_.col(idx_cur);
            Eigen::Vector3d f_ref = frame_ref->f_vec_.col(idx_ref);

            Eigen::Vector3d norm_cur = f_cur / f_cur.z();
            Eigen::Vector3d norm_ref = f_ref / f_ref.z();
            Eigen::Vector3d pts_cam_ref = norm_ref * depth_ref;

            cv::Point2f pt_2d_cur(norm_cur.x(), norm_cur.y());
            cv::Point3f pt_3d_ref(pts_cam_ref.x(), pts_cam_ref.y(), pts_cam_ref.z());

            cur_matched_norm.emplace_back(pt_2d_cur);
            ref_matched_3d.emplace_back(pt_3d_ref);
        }

        if (ref_matched_3d.size() > init_3d_points_size)
        {
            int inlier_size = PnPRansac(cur_matched_norm, ref_matched_3d, T_cur_ref);
            if (inlier_size > init_3d_points_size / 2)
                return true;
        }
        else
            printf("The number of 3d point from ref depth image (%d) is too little, less than 25\n",
                   (int)ref_matched_3d.size());

        // Case 2: get T_cur_ref with 3D points(cur frame) & 2D points(ref frame)
        std::vector<cv::Point3f> cur_matched_3d;
        std::vector<cv::Point2f> ref_matched_norm;

        for (auto &it : matches_cur_ref)
        {
            size_t idx_cur = it.first;
            size_t idx_ref = it.second;

            Eigen::Vector2d px_cur = frame_cur->px_vec_.col(idx_cur);
            int x_cur = round(px_cur.x()), y_cur = round(px_cur.y());
            float depth_cur = frame_cur->GetValidDepthFromImage(y_cur, x_cur);

            if (depth_cur < depth_img_min_ || depth_cur > depth_img_max_)
                continue;

            Eigen::Vector3d f_cur = frame_cur->f_vec_.col(idx_cur);
            Eigen::Vector3d f_ref = frame_ref->f_vec_.col(idx_ref);

            Eigen::Vector3d norm_cur = f_cur / f_cur.z();
            Eigen::Vector3d norm_ref = f_ref / f_ref.z();

            Eigen::Vector3d pts_cam_cur = norm_cur * depth_cur;

            cv::Point3f pt_3d_cur(pts_cam_cur.x(), pts_cam_cur.y(), pts_cam_cur.z());
            cv::Point2f pt_2d_ref(norm_ref.x(), norm_ref.y());

            cur_matched_3d.emplace_back(pt_3d_cur);
            ref_matched_norm.emplace_back(pt_2d_ref);
        }

        if (cur_matched_3d.size() > init_3d_points_size)
        {
            Transformation T_ref_cur;
            int inlier_size = PnPRansac(ref_matched_norm, cur_matched_3d, T_ref_cur);
            if (inlier_size > init_3d_points_size / 2)
            {
                T_cur_ref = T_ref_cur.Inverse();
                return true;
            }
        }
        else
            printf("The number of 3d point from cur depth image (%d) is too little, less than 25\n",
                   (int)cur_matched_3d.size());

        return false;
    }

    // void ChannelFrameRgbdFisheye::trackNewFeatures(const FrameBundlePtr& frames)
    // {
    //   tracker_->resetActiveTracks();
    //   for(const FramePtr& frame : frames->frames_)
    //     frame->ClearFeatureStorage();
    //   const size_t n = tracker_->initializeNewTracks(frames);
    // }

    bool ChannelFrameRgbdFisheye::TriangulateAndInitializePoints(
        const FrameBundlePtr &frames_cur, const FrameBundlePtr &frames_ref)
    {
        std::vector<std::unordered_map<size_t, size_t>> matches_cur_ref_vec;
        std::vector<std::unordered_map<size_t, cv::Point3f>> points_in_world_vec;

        for (size_t i = 0; i < frames_cur->size(); ++i)
        {
            const FramePtr frame_cur = frames_cur->at(i);
            const FramePtr frame_ref = frames_ref->at(i);

            std::unordered_map<size_t, size_t> matches_cur_ref;
            //getFeatureMatches(frame_cur, frame_ref, matches_cur_ref);
            //drawFeatureMatches(frame_cur, frame_ref, matches_cur_ref);
            feature_tracker_tools::GetFeatureMatches(frame_cur, frame_ref, matches_cur_ref);
            feature_tracker_tools::DrawFeatureMatches(frame_cur, frame_ref, matches_cur_ref);

            if (matches_cur_ref.size() < 8)
                return false;

            RejectWithF(frame_cur, frame_ref, matches_cur_ref); //F矩阵剔除外点，更新matches_cur_ref

            //三角化
            std::unordered_map<size_t, cv::Point3f> points_in_world;
            TriangulatePoints(frame_cur, frame_ref, matches_cur_ref, points_in_world);
            std::cout << "Camera: " << i << ", 3D point size: " << points_in_world.size() << std::endl;

            if (i == 0 && (int)points_in_world.size() < init_landmark_size_)
                return false;
            if ((int)points_in_world.size() < init_landmark_size_)
                return false;
            matches_cur_ref_vec.emplace_back(matches_cur_ref);
            points_in_world_vec.emplace_back(points_in_world);
        }

        for (size_t i = 0; i < frames_cur->size(); ++i)
        {
            const FramePtr frame_cur = frames_cur->at(i);
            const FramePtr frame_ref = frames_ref->at(i);

            auto &matches_cur_ref = matches_cur_ref_vec[i];
            auto &points_in_world = points_in_world_vec[i];

            //设置世界点landmark
            InitializePoints(frame_cur, frame_ref, matches_cur_ref, points_in_world);
        }

        return true;
    }

    void ChannelFrameRgbdFisheye::TriangulatePoints(
        const FramePtr &frame_cur, const FramePtr &frame_ref,
        std::unordered_map<size_t, size_t> &matches_cur_ref,
        std::unordered_map<size_t, cv::Point3f> &points_in_world)
    {
        Transformation T_world_cur = frame_cur->T_world_cam();
        Transformation T_world_ref = frame_ref->T_world_cam();
        Transformation T_cur_ref = T_world_cur.Inverse() * T_world_ref;
        Transformation T_ref_cur = T_cur_ref.Inverse();

        std::cout << "*************************\n";
        std::cout << "T_ref_cur: \n " << T_ref_cur << std::endl;
        std::cout << "*************************\n";
        std::cout << "frame_cur pose\n"
                  << frame_cur->T_world_cam() << std::endl;
        std::cout << "frame_ref pose\n"
                  << frame_ref->T_world_cam() << std::endl;
        std::cout << "*************************\n";

        const double angle_threshold = frame_cur->GetAngleError(reproj_error_thresh_);

        Eigen::VectorXd intrinsic = frame_cur->cam_->getIntrinsicParameters();
        float focal_length = std::sqrt(intrinsic(0) * intrinsic(1));

        Eigen::Matrix<double, 3, 4> pose_cur, pose_ref;
        pose_cur = frame_cur->T_cam_world().GetTransformationMatrix().block<3, 4>(0, 0);
        pose_ref = frame_ref->T_cam_world().GetTransformationMatrix().block<3, 4>(0, 0);

        std::unordered_map<size_t, size_t> matches_cur_ref_new;
        for (auto &it : matches_cur_ref)
        {
            size_t idx_cur = it.first;
            size_t idx_ref = it.second;
            BearingVector f_cur = frame_cur->f_vec_.col(idx_cur);
            BearingVector f_ref = frame_ref->f_vec_.col(idx_ref);

            Eigen::Vector3d norm_pt_cur = f_cur / f_cur.z();
            Eigen::Vector3d norm_pt_ref = f_ref / f_ref.z();

            Eigen::Vector3d pt_cur = T_cur_ref.GetRotationMatrix() * norm_pt_ref;
            float cos_parallax_rays = norm_pt_cur.dot(pt_cur) / (norm_pt_cur.norm() * pt_cur.norm());
            if (cos_parallax_rays > 0.9997)
                continue;

            Position xyz_in_cur = vk::triangulateFeatureNonLin(
                T_cur_ref.GetRotationMatrix(), T_cur_ref.GetPosition(), f_cur, f_ref);
            Position xyz_in_ref = T_ref_cur * xyz_in_cur;
            Eigen::Vector3d xyz_in_world = frame_ref->T_world_cam() * xyz_in_ref;

            // Eigen::Vector3d pts_world = Eigen::Vector3d::Identity();
            // triangulatePoint(pose_cur, pose_ref, norm_pt_cur, norm_pt_ref, xyz_in_world);

            // Eigen::Vector3d pts_cam_cur = pose_cur.block<3, 3>(0, 0) * pts_world + pose_cur.block<3, 1>(0, 3);
            // Eigen::Vector3d pts_cam_ref = pose_ref.block<3, 3>(0, 0) * pts_world + pose_ref.block<3, 1>(0, 3);

            if (frame_cur->cam()->GetType() == Camera::Type::kPinhole && (xyz_in_cur.z() < 0.0f || xyz_in_ref.z() < 0.0f))
                continue;

            // const double e_cur = std::acos(f_cur.dot(xyz_in_cur)  / (f_cur.norm() * xyz_in_cur.norm()));
            // const double e_ref = std::acos(f_ref.dot(xyz_in_ref)  / (f_ref.norm() * xyz_in_ref.norm()));
            // if(e_cur > angle_threshold || e_ref > angle_threshold)
            //   continue;

            Eigen::Vector3d norm_proj_cur = xyz_in_cur / xyz_in_cur.z();
            Eigen::Vector3d norm_proj_ref = xyz_in_ref / xyz_in_ref.z();
            float norm_error_cur = (norm_proj_cur - norm_pt_cur).norm();
            float norm_error_ref = (norm_proj_ref - norm_pt_ref).norm();
            if (norm_error_cur > 1.0f / focal_length || norm_error_ref > 1.0f / focal_length)
                continue;

            {
                if (!frame_cur->depth_image_.empty())
                {
                    Eigen::Vector2d px = frame_cur->px_vec_.col(idx_cur);
                    int x = std::round(px.x()), y = std::round(px.y());
                    float depth = frame_cur->GetValidDepthFromImage(y, x);
                    std::cout << depth << "; " << xyz_in_cur.z() << "\n"; // << pts_cam_cur.z() << "\n";
                }
                else
                    std::cout << xyz_in_cur.z() << "\n"; // << pts_cam_cur.z() << "\n";
            }

            cv::Point3f point_world = cv::Point3f(xyz_in_world.x(), xyz_in_world.y(), xyz_in_world.z());
            matches_cur_ref_new[idx_cur] = idx_ref;
            points_in_world[idx_cur] = point_world;
        }

        matches_cur_ref = matches_cur_ref_new;
    }

    void ChannelFrameRgbdFisheye::InitializePoints(
        const FramePtr &frame_cur, const FramePtr &frame_ref,
        const std::unordered_map<size_t, size_t> &matches_cur_ref,
        const std::unordered_map<size_t, cv::Point3f> &points_in_world)
    {
        CHECK_EQ(matches_cur_ref.size(), points_in_world.size());

        for (auto &it : matches_cur_ref)
        {
            int idx_cur = it.first;
            int idx_ref = it.second;
            cv::Point3f pt_world = points_in_world.at(idx_cur);

            const Eigen::Vector3d xyz_in_world(pt_world.x, pt_world.y, pt_world.z);
            const int point_id_cur = frame_cur->track_id_vec_(idx_cur);
            const int point_id_ref = frame_ref->track_id_vec_(idx_ref);

            CHECK_EQ(point_id_cur, point_id_ref);

            PointPtr new_point(new Point(point_id_cur, xyz_in_world));
            frame_cur->landmark_vec_.at(idx_cur) = new_point;
            frame_ref->landmark_vec_.at(idx_ref) = new_point;
            new_point->AddObservation(frame_ref, idx_ref);
            new_point->AddObservation(frame_cur, idx_cur);
            new_point->SetConstant();
        }
    }

    void ChannelFrameRgbdFisheye::RejectWithF(
        const FramePtr &frame1, const FramePtr &frame2,
        std::unordered_map<size_t, size_t> &matches_12)
    {
        std::vector<cv::Point2f> un_pts1;
        std::vector<cv::Point2f> un_pts2;

        for (auto &iter : matches_12)
        {
            size_t idx1 = iter.first;
            size_t idx2 = iter.second;

            Eigen::Vector3d pts1 = frame1->f_vec_.col(idx1);
            cv::Point2f pt1 = cv::Point2f(pts1.x() / pts1.z(),
                                          pts1.y() / pts1.z());
            un_pts1.emplace_back(pt1);

            Eigen::Vector3d pts2 = frame2->f_vec_.col(idx2);
            cv::Point2f pt2 = cv::Point2f(pts2.x() / pts2.z(),
                                          pts2.y() / pts2.z());
            un_pts2.emplace_back(pt2);
        }

        std::vector<uchar> status;
        cv::findFundamentalMat(un_pts1, un_pts2, cv::FM_RANSAC, 1.0 / 460, 0.99, status);

        int idx = 0;
        std::unordered_map<size_t, size_t> matches_12_new;
        for (auto &iter : matches_12)
        {
            size_t idx1 = iter.first;
            size_t idx2 = iter.second;

            if (status[idx])
                matches_12_new[idx1] = idx2;

            ++idx;
        }

        matches_12 = matches_12_new;
    }

    UpdateResult ChannelFrameRgbdFisheye::ProcessVinsFrame()
    {
        std::vector<FramePtr> kfs_sorted;
        map_->GetSortedKeyframes(kfs_sorted);
        std::cout << "size=" << kfs_sorted.size() << std::endl;
        for (const FramePtr &frame : kfs_sorted)
        {
            frame->SetKeyframe();
            if (!frame->depth_image_.empty())
            {
                frame_utils::GetSceneDepth(frame, depth_median_, depth_min_, depth_max_);
            }
            depth_filter_->AddKeyframe(frame, depth_median_, 0.5 * depth_min_, depth_median_ * 1.5); //DEPTHTODO
        }

        stage_ = Stage::kTracking;
        tracking_quality_ = TrackingQuality::kGood;
        return UpdateResult::kKeyframe;
    }
    UpdateResult ChannelFrameRgbdFisheye::ProcessFrame()
    {
        GetMotionPriorWithDepth();
        VLOG(40) << "===== Sparse Image Alignment =====";
        size_t n_total_observations = 0;
        SparseImageAlignment();
        VLOG(40) << "===== Project FrontendLocalMap to Current Frame =====";
        n_total_observations = ProjectMapInFrame();
        if (n_total_observations < options_.quality_min_fts)
        {
            LOG(WARNING) << "Not enough feature after reprojection: "
                         << n_total_observations;
            // return UpdateResult::kFailure;
            for (size_t i = 0; i < new_frames_->size(); ++i)
            {
                MakeKeyframe(i);
            }
            return UpdateResult::kKeyframe;
        }
        // redundant when using ceres backend
        VLOG(40) << "===== Pose Optimization =====";
        n_total_observations = OptimizePose(); //位姿
        if (n_total_observations < options_.quality_min_fts)
        {
            LOG(WARNING) << "Not enough feature after pose optimization."
                         << n_total_observations;
            // return UpdateResult::kFailure;
            for (size_t i = 0; i < new_frames_->size(); ++i)
            {
                MakeKeyframe(i);
            }
            return UpdateResult::kKeyframe;
        }
        OptimizeStructure(new_frames_, options_.structure_optimization_max_pts, 5); //投标点

        SetTrackingQuality(n_total_observations); //check是否追踪发散
        if (tracking_quality_ == TrackingQuality::kInsufficient)
        {
            // return UpdateResult::kFailure;
            for (size_t i = 0; i < new_frames_->size(); ++i)
            {
                MakeKeyframe(i);
            }
            return UpdateResult::kKeyframe;
        }
        // select keyframe
        VLOG(40) << "===== Keyframe Selection =====";

        bool bUseRlocInSvo = false; //TODO param
        if (bUseRlocInSvo)
        {
            if (!need_new_kf_(new_frames_->at(0)->T_f_w_) || tracking_quality_ == TrackingQuality::kBad || stage_ == Stage::kRelocalization)
            {
                if (tracking_quality_ == TrackingQuality::kGood)
                {
                    VLOG(40) << "Updating seeds in overlapping keyframes...";
                    CHECK(!overlap_kfs_.empty());

                    //depth_filter_->SeedsUpdate(overlap_kfs_.at(0), new_frames_->frames_[0]);//TODO
                    for (size_t bn = 0; bn < new_frames_->size(); bn++)
                    {
                        depth_filter_->SeedsUpdate({new_frames_->frames_[bn]}, last_frames_->frames_[bn]);
                        for (const FramePtr &old_keyframe : overlap_kfs_.at(bn))
                            depth_filter_->SeedsUpdate({new_frames_->frames_[bn]}, old_keyframe);
                    }
                }
                return UpdateResult::kDefault;
            }
        }
        else
        {
            if (!need_new_kf_(new_frames_->at(0)->T_f_w_))
            {
                for (size_t i = 0; i < new_frames_->size(); ++i)
                {
                    depth_filter_->SeedsUpdate(overlap_kfs_.at(i), new_frames_->at(i));
                }
                return UpdateResult::kDefault;
            }
        }

        for (size_t i = 0; i < new_frames_->size(); ++i)
        {
            MakeKeyframe(i);
        }
        return UpdateResult::kKeyframe;
    }

    UpdateResult ChannelFrameRgbdFisheye::MakeKeyframe(const size_t camera_id)
    {

        new_frames_->frames_[camera_id]->SetKeyframe(); //添加新的关键帧

        VLOG(40) << "New keyframe selected.";
        UpgradeSeedsToFeatures(new_frames_->frames_[camera_id]);
        {
            DepthOptimization::ULockT lock(depth_filter_->m_feature_detector_mut);
            SetDetectorOccupiedCells(camera_id, depth_filter_->m_feature_detector);
        } // release lock
        frame_utils::GetSceneDepth(new_frames_->frames_[camera_id], depth_median_, depth_min_, depth_max_);

        // if(!new_frames_->frames_[camera_id]->depth_image_.empty() ) {
        //   depth_filter_->AddKeyframe(
        //       new_frames_->frames_[camera_id], depth_median_, 0.8*depth_min_, depth_median_*1.2);  //0.5*   *1.5 DEPTHTODO
        // }
        // else
        {
            depth_filter_->AddKeyframe(
                new_frames_->frames_[camera_id], depth_median_, 0.5 * depth_min_, depth_median_ * 1.5); //0.5*   *1.5 DEPTHTODO
        }

        VLOG(40) << "Updating seeds in overlapping keyframes...";

        // add keyframe to map
        map_->AddKeyframe(new_frames_->frames_[camera_id], true);

        depth_filter_->SeedsUpdate(overlap_kfs_.at(camera_id), new_frames_->frames_[camera_id]); //TODO

        // if limited number of keyframes, remove the one furthest apart
        if (options_.max_n_kfs > 2)
        {
            while (map_->size() > options_.max_n_kfs)
            {
                map_->RemoveOldestKeyframe();
            }
        }
        return UpdateResult::kKeyframe;
    }

    UpdateResult ChannelFrameRgbdFisheye::ProcessFrameMonoVersion()
    {
        VLOG(40) << "Updating seeds in overlapping keyframes...";

        // tracking重点：前端跟踪三个Step

        // STEP 1: Sparse Image Align
        VLOG(40) << "===== Sparse Image Alignment =====";
        size_t n_total_observations = 0;
        SparseImageAlignment(); //优化Tbk_bk-1

        // STEP 2: FrontendLocalMap Reprojection & Feature Align 重投影和特征点匹配
        VLOG(40) << "===== Project FrontendLocalMap to Current Frame =====";
        n_total_observations = ProjectMapInFrame();
        if (n_total_observations < options_.quality_min_fts)
        {
            LOG(WARNING) << "Not enough feature after reprojection: "
                         << n_total_observations;
            return UpdateResult::kFailure;
        }

        // STEP 3: Pose & Structure Optimization  位姿与特征点优化
        // redundant when using ceres backend
        if (bundle_adjustment_type_ != BundleAdjustmentType::kCeres)
        {
            VLOG(40) << "===== Pose Optimization =====";
            n_total_observations = OptimizePose(); //位姿
            if (n_total_observations < options_.quality_min_fts)
            {
                LOG(WARNING) << "Not enough feature after pose optimization."
                             << n_total_observations;
                return UpdateResult::kFailure;
            }
            OptimizeStructure(new_frames_, options_.structure_optimization_max_pts, 5); //投标点
        }

        // return if tracking bad
        SetTrackingQuality(n_total_observations); //check是否追踪发散
        if (tracking_quality_ == TrackingQuality::kInsufficient)
            return UpdateResult::kFailure;

        // ---------------------------------------------------------------------------
        // select keyframe
        VLOG(40) << "===== Keyframe Selection =====";
        frame_utils::GetSceneDepth(new_frames_->frames_[0], depth_median_, depth_min_, depth_max_);
        if (!need_new_kf_(new_frames_->frames_[0]->T_f_w_) || tracking_quality_ == TrackingQuality::kBad || stage_ == Stage::kRelocalization)
        {
            if (tracking_quality_ == TrackingQuality::kGood)
            {
                VLOG(40) << "Updating seeds in overlapping keyframes...";
                CHECK(!overlap_kfs_.empty());

                //depth_filter_->SeedsUpdate(overlap_kfs_.at(0), new_frames_->frames_[0]);//TODO
                for (size_t bn = 0; bn < new_frames_->size(); bn++)
                {
                    depth_filter_->SeedsUpdate({new_frames_->frames_[bn]}, last_frames_->frames_[bn]);
                    for (const FramePtr &old_keyframe : overlap_kfs_.at(bn))
                        depth_filter_->SeedsUpdate({new_frames_->frames_[bn]}, old_keyframe);
                }
            }
            for (size_t bn = 0; bn < new_frames_->size(); bn++)
            {
                depth_filter_->SeedsUpdate(overlap_kfs_.at(bn), new_frames_->frames_[bn]);
            }
            return UpdateResult::kDefault;
        }
        // new_frames_->frames_[0]->SetKeyframe();//添加新的关键帧
        for (size_t bn = 0; bn < new_frames_->size(); bn++)
        {
            new_frames_->frames_[bn]->SetKeyframe(); //添加新的关键帧
        }
        VLOG(40) << "New keyframe selected.";
        //新的关键帧：新提取一些seed并不断循环直到投影到该帧进行更新
        // ---------------------------------------------------------------------------
        // new keyframe selected
        if (depth_filter_->m_opt_options.extra_map_points)
        {
            depth_filter_->m_sec_feature_detector->ResetGrid();

            for (size_t bn = 0; bn < new_frames_->size(); bn++)
            {
                OccupandyGrid2D map_point_grid(depth_filter_->m_sec_feature_detector->grid_);
                reprojector_utils::ReprojectMapPoints(new_frames_->frames_[bn], overlap_kfs_.at(bn),
                                                      reprojectors_.at(bn)->options_, &map_point_grid);
                DepthOptimization::ULockT lock(depth_filter_->m_feature_detector_mut);
                feature_detector_utils::MergeGrids(
                    map_point_grid, &depth_filter_->m_sec_feature_detector->grid_);
            }
        }
        for (size_t bn = 0; bn < new_frames_->size(); bn++)
        {
            UpgradeSeedsToFeatures(new_frames_->frames_[bn]);
            {
                DepthOptimization::ULockT lock(depth_filter_->m_feature_detector_mut);
                SetDetectorOccupiedCells(bn, depth_filter_->m_feature_detector);

            } // release lock
            frame_utils::GetSceneDepth(new_frames_->frames_[bn], depth_median_, depth_min_, depth_max_);
            if (!new_frames_->frames_[bn]->depth_image_.empty())
            {
                depth_filter_->AddKeyframe(
                    new_frames_->frames_[bn], depth_median_, 0.5 * depth_min_, depth_median_ * 1.5); //0.5*   *1.5 DEPTHTODO
            }
            else
            {
                depth_filter_->AddKeyframe(
                    new_frames_->frames_[bn], depth_median_, 0.5 * depth_min_, depth_median_ * 1.5); //0.5*   *1.5 DEPTHTODO
            }
            if (options_.update_seeds_with_old_keyframes) // TODO: options_.update_seeds_with_old_keyframes     when cam:2 null
            {
                VLOG(40) << "Updating seeds in current frame using last frame...";
                depth_filter_->SeedsUpdate({new_frames_->frames_[bn]}, last_frames_->frames_[bn]);
                VLOG(40) << "Updating seeds in current frame using overlapping keyframes...";
                for (const FramePtr &old_keyframe : overlap_kfs_.at(bn))
                    depth_filter_->SeedsUpdate({new_frames_->frames_[bn]}, old_keyframe);
            }
            VLOG(40) << "Updating seeds in overlapping keyframes...";
            //  depth_filter_->SeedsUpdate(overlap_kfs_.at(0), new_frames_->frames_[0]);

            // add keyframe to map
            map_->AddKeyframe(new_frames_->frames_[bn], bundle_adjustment_type_ == BundleAdjustmentType::kCeres);

            depth_filter_->SeedsUpdate(overlap_kfs_.at(bn), new_frames_->frames_[bn]); //TODO
        }
        // if limited number of keyframes, remove the one furthest apart
        if (options_.max_n_kfs > 2)
        {
            while (map_->size() > options_.max_n_kfs)
            {
                map_->RemoveOldestKeyframe();
                map_->RemoveOldestKeyframe();
                map_->RemoveOldestKeyframe();
            }
        }
        return UpdateResult::kKeyframe;
    }

    /*reloc use bundle data*/
    UpdateResult ChannelFrameRgbdFisheye::RelocalizeFramebundle(
        const Transformation & /*T_cur_ref*/,
        const FrameBundlePtr &ref_keyframe)
    {
        ++relocalization_n_trials_;
        if (ref_keyframe == nullptr)
            return UpdateResult::kFailure;

        VLOG_EVERY_N(1, 20) << "Relocalizing frame";
        FrameBundle::Ptr ref_frame(new FrameBundle({ref_keyframe->frames_}, ref_keyframe->GetBundleId()));

        last_frames_ = ref_frame;
        UpdateResult res = ProcessFrame();
        if (res == UpdateResult::kDefault)
        {
            // Reset to default mode.
            stage_ = Stage::kTracking;
            relocalization_n_trials_ = 0;
            VLOG(1) << "Relocalization successful.......";
            std::cout << "relocalization successful ... ... " << std::endl;
        }
        else
        {
            // reset to last well localized pose
            new_frames_->frames_[0]->T_f_w_ = (last_frames_->Get_T_W_B() * last_frames_->frames_[0]->T_body_cam_).Inverse();
            new_frames_->frames_[1]->T_f_w_ = (last_frames_->Get_T_W_B() * last_frames_->frames_[1]->T_body_cam_).Inverse();
            new_frames_->frames_[2]->T_f_w_ = (last_frames_->Get_T_W_B() * last_frames_->frames_[2]->T_body_cam_).Inverse();
            std::cout << "relocalization not successful ... ... " << std::endl;
        }
        return res;
    }

    UpdateResult ChannelFrameRgbdFisheye::RelocalizeFrame(
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
            new_frames_->frames_[0]->T_f_w_ = ref_keyframe->T_f_w_;
            // new_frames_->frames_[1]->T_f_w_ = ref_keyframe->T_f_w_;
            // new_frames_->frames_[2]->T_f_w_ = ref_keyframe->T_f_w_;
        }
        return res;
    }

    void ChannelFrameRgbdFisheye::ResetAll()
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

    void ChannelFrameRgbdFisheye::GetMotionPriorWithDepth()
    {
        if (!options_.use_imu_only_for_gravityalign)
        {
            if (imu_handler_)
            {
                return;
            }
        }
        if (last_frames_ == nullptr)
            return;

        int inlier_thresh = 25;
        float rotation_thresh = 10.0f;

        int depth_cam_id = 0;
        const FramePtr &frame_cur = new_frames_->at(depth_cam_id);
        const FramePtr &frame_ref = last_frames_->at(depth_cam_id);

        FeatureTracksPerCam tracks;
        tracks.reserve(frame_ref->NumFeatures());
        for (size_t feat_idx = 0; feat_idx < frame_ref->NumFeatures(); ++feat_idx)
        {
            if (feat_idx >= (size_t)frame_ref->track_id_vec_.size())
                break;
            int track_id = frame_ref->track_id_vec_[feat_idx];
            tracks.emplace_back(track_id);
            tracks.back().pushBack(last_frames_, depth_cam_id, feat_idx);
        }

        size_t match_cnt = 0;
        Keypoints px_vec_ref(2, tracks.size());
        Keypoints px_vec_cur(2, tracks.size());
        for (size_t track_index = 0; track_index < tracks.size(); ++track_index)
        {
            FeatureTrackPerId &track = tracks.at(track_index);

            const ImgPyramid &pyr_ref = frame_ref->img_pyr_;
            const ImgPyramid &pyr_cur = frame_cur->img_pyr_;

            Keypoint ref_px_level_0 = track.back().GetPx();
            Keypoint cur_px_level_0 = track.back().GetPx();
            bool success = feature_patch_alignment::AlignPyr2D(
                pyr_ref, pyr_cur,
                tracker_options_.klt_max_level, tracker_options_.klt_min_level,
                tracker_options_.klt_patch_sizes, tracker_options_.klt_max_iter,
                tracker_options_.klt_min_update_squared,
                ref_px_level_0.cast<int>(), cur_px_level_0);
            if (success)
            {
                px_vec_ref.col(match_cnt) = ref_px_level_0;
                px_vec_cur.col(match_cnt) = cur_px_level_0;
                ++match_cnt;
            }
        }
        px_vec_ref.conservativeResize(Eigen::NoChange, match_cnt);
        px_vec_cur.conservativeResize(Eigen::NoChange, match_cnt);

        Bearings f_vec_ref, f_vec_cur;
        frame_utils::ComputeNormalizedBearingVectors(px_vec_ref, *frame_ref->cam(), &f_vec_ref);
        frame_utils::ComputeNormalizedBearingVectors(px_vec_cur, *frame_cur->cam(), &f_vec_cur);

        std::vector<cv::Point2f> norm_pts_ref;
        std::vector<cv::Point2f> norm_pts_cur;
        for (int i = 0; i < px_vec_cur.cols(); ++i)
        {
            Eigen::Vector3d f_ref = f_vec_ref.col(i);
            cv::Point2f pt_ref(f_ref.x() / f_ref.z(), f_ref.y() / f_ref.z());
            norm_pts_ref.emplace_back(pt_ref);

            Eigen::Vector3d f_cur = f_vec_cur.col(i);
            cv::Point2f pt_cur(f_cur.x() / f_cur.z(), f_cur.y() / f_cur.z());
            norm_pts_cur.emplace_back(pt_cur);
        }

        if (norm_pts_cur.size() < 8)
            return;

        std::vector<uchar> status;
        Eigen::VectorXd intrinsic = frame_cur->cam_->getIntrinsicParameters();
        float focal_length = std::sqrt(intrinsic(0) * intrinsic(1));
        cv::findFundamentalMat(norm_pts_ref, norm_pts_cur, cv::FM_RANSAC, 1.0 / focal_length, 0.99, status);

        std::vector<cv::Point2f> matched_2d_cur;
        std::vector<cv::Point3f> matched_3d_ref;
        for (int i = 0; i < px_vec_ref.cols(); ++i)
        {
            if (!status[i])
                continue;

            Eigen::Vector2d px_ref = px_vec_ref.col(i);
            int x_ref = std::round(px_ref.x()), y_ref = std::round(px_ref.y());
            float depth_ref = frame_ref->GetValidDepthFromImage(y_ref, x_ref);

            if (depth_ref < depth_img_min_ || depth_ref > depth_img_max_)
                continue;

            Eigen::Vector3d f_ref = f_vec_ref.col(i);
            Eigen::Vector3d f_cur = f_vec_cur.col(i);

            Eigen::Vector3d norm_pt_ref = f_ref / f_ref.z();
            Eigen::Vector3d norm_pt_cur = f_cur / f_cur.z();

            Eigen::Vector3d pt_cam_ref = norm_pt_ref * depth_ref;

            cv::Point3f pt_3d_ref(pt_cam_ref.x(), pt_cam_ref.y(), pt_cam_ref.z());
            cv::Point2f pt_2d_cur(norm_pt_cur.x(), norm_pt_cur.y());

            matched_3d_ref.emplace_back(pt_3d_ref);
            matched_2d_cur.emplace_back(pt_2d_cur);
        }

        int inlier_size = 0;
        Transformation T_curC_refC;
        inlier_size = PnPRansac(matched_2d_cur, matched_3d_ref, T_curC_refC);
        if (inlier_size > inlier_thresh)
        {
            Transformation &T_c_b = frame_cur->T_cam_body_;
            Transformation &T_b_c = frame_cur->T_body_cam_;
            Transformation T_curB_refB = T_b_c * T_curC_refC * T_c_b;
            Transformation T_w_refB = last_frames_->Get_T_W_B();
            Transformation T_w_curB = T_w_refB * T_curB_refB.Inverse();

            Eigen::AngleAxisd rotation_vector;
            rotation_vector.fromRotationMatrix(T_curB_refB.Inverse().GetRotationMatrix());

            if (rotation_vector.angle() * 180 / M_PI < rotation_thresh)
            {
                T_newimu_lastimu_prior_ = T_curB_refB;
                new_frames_->Set_T_W_B(T_w_curB);
                have_motion_prior_ = true;

                R_imu_world_ = new_frames_->Get_T_W_B().Inverse().GetRotation();
                have_rotation_prior_ = true;

                // std::cout << "Motion Prior: \n" << T_curB_refB.Inverse() << std::endl;
                return;
            }
        }

        std::vector<cv::Point2f> matched_2d_ref;
        std::vector<cv::Point3f> matched_3d_cur;
        for (int i = 0; i < px_vec_cur.cols(); ++i)
        {
            if (!status[i])
                continue;

            Eigen::Vector2d px_cur = px_vec_cur.col(i);
            int x_cur = std::round(px_cur.x()), y_cur = std::round(px_cur.y());
            float depth_cur = frame_cur->GetValidDepthFromImage(y_cur, x_cur);

            if (depth_cur < depth_img_min_ || depth_cur > depth_img_max_)
                continue;

            Eigen::Vector3d f_ref = f_vec_ref.col(i);
            Eigen::Vector3d f_cur = f_vec_cur.col(i);

            Eigen::Vector3d norm_pt_ref = f_ref / f_ref.z();
            Eigen::Vector3d norm_pt_cur = f_cur / f_cur.z();

            Eigen::Vector3d pt_cam_cur = norm_pt_cur * depth_cur;

            cv::Point3f pt_3d_cur(pt_cam_cur.x(), pt_cam_cur.y(), pt_cam_cur.z());
            cv::Point2f pt_2d_ref(norm_pt_ref.x(), norm_pt_ref.y());

            matched_3d_cur.emplace_back(pt_3d_cur);
            matched_2d_ref.emplace_back(pt_2d_ref);
        }

        Transformation T_refC_curC;
        inlier_size = PnPRansac(matched_2d_ref, matched_3d_cur, T_refC_curC);
        if (inlier_size > inlier_thresh)
        {
            Transformation &T_c_b = frame_cur->T_cam_body_;
            Transformation &T_b_c = frame_cur->T_body_cam_;
            Transformation T_refB_curB = T_b_c * T_refC_curC * T_c_b;
            Transformation T_w_refB = last_frames_->Get_T_W_B();
            Transformation T_w_curB = T_w_refB * T_refB_curB;

            Eigen::AngleAxisd rotation_vector;
            rotation_vector.fromRotationMatrix(T_refB_curB.GetRotationMatrix());

            if (rotation_vector.angle() * 180 / M_PI < rotation_thresh)
            {
                T_newimu_lastimu_prior_ = T_refB_curB.Inverse();
                new_frames_->Set_T_W_B(T_w_curB);
                have_motion_prior_ = true;

                R_imu_world_ = new_frames_->Get_T_W_B().Inverse().GetRotation();
                have_rotation_prior_ = true;
                // std::cout << "Motion Prior: \n" << T_refB_curB.Inverse() << std::endl;
                return;
            }
        }
    }
    int ChannelFrameRgbdFisheye::PnPRansac(
        const std::vector<cv::Point2f> &matched_2d,
        const std::vector<cv::Point3f> &matched_3d,
        Transformation &T_2d_3d)
    {
        if (matched_3d.size() < 4)
            return 0;

        cv::Mat inliers;
        cv::Mat r, rvec, t, D, tmp_r;
        cv::Mat K = cv::Mat::eye(3, 3, CV_32FC1);

        Eigen::VectorXd intrinsic = cams_->getCamera(0).getIntrinsicParameters();
        float focal_length = std::sqrt(intrinsic(0) * intrinsic(1));
        ; //std::sqrt(intrinsic(0) * intrinsic(1));

        std::cout << "matched 3d size: " << matched_3d.size() << std::endl;

        if (CV_MAJOR_VERSION < 3)
        {
            cv::solvePnPRansac(matched_3d, matched_2d, K, D, rvec,
                               t, true, 100, 1.0 / focal_length, 100, inliers);
        }
        else
        {
            if (CV_MINOR_VERSION < 2)
                cv::solvePnPRansac(matched_3d, matched_2d, K, D, rvec,
                                   t, true, 100, sqrt(1.0 / focal_length), 0.99, inliers);
            else
                cv::solvePnPRansac(matched_3d, matched_2d, K, D, rvec,
                                   t, true, 100, 1.0 / focal_length, 0.99, inliers);
        }

        Eigen::Matrix3d R_2d_3d;
        Eigen::Vector3d t_2d_3d;
        cv::Rodrigues(rvec, r);
        cv::cv2eigen(r, R_2d_3d);
        cv::cv2eigen(t, t_2d_3d);

        T_2d_3d.setIdentity();
        T_2d_3d.GetRotation() = Transformation::Rotation(R_2d_3d);
        T_2d_3d.GetPosition() = t_2d_3d;

        return (int)inliers.rows;
    }

} // namespace mivins
