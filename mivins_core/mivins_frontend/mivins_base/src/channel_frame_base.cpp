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

#include "mivins/channel_frame_base.h"

#include <functional>
#include <future>
#include <memory>
#include <opencv2/imgproc/imgproc.hpp>

#include <mivins/common/conversions.h>
#include <mivins/common/point.h>
#include <mivins/common/imu_calibration.h>
#include <mivins/direct/depth_optimization.h>
#include <mivins/direct/feature_detector.h>
#include <mivins/direct/feature_detector_utilities.h>
#include <mivins/direct/patch_matcher.h>
#include <mivins/tracker/feature_tracker.h>


#ifdef SVO_GLOBAL_MAP
#include <mivins/global_map.h>
#endif

#include <mivins/img_align/sparse_img_align.h>

//#include "svo/abstract_bundle_adjustment.h"
#include <mivins/bundle_adjustment.h>
#include "mivins/initialization.h"
#include "mivins/frontend_local_map.h"
#include "mivins/reprojector.h"
#include "mivins/channel_imu.h"
#include "mivins/channel_odom.h"
#include "mivins/pose_optimizer.h"

namespace
{
    inline double DistanceFirstTwoKeyframes(mivins::FrontendLocalMap &frontend_map)
    {
        mivins::FramePtr first_kf = frontend_map.GetKeyFrameAt(0);
        mivins::FramePtr second_kf = frontend_map.GetKeyFrameAt(1);
        double dist =
            (first_kf->T_world_imu().GetPosition() -
             second_kf->T_world_imu().GetPosition())
                .norm();
        return dist;
    }
}

namespace mivins
{

    // definition of global and static variables which were declared in the header
    PerformanceMonitorPtr g_permon;

    const std::unordered_map<mivins::Stage, std::string, EnumClassHash> kStageName{{Stage::kPaused, "Paused"}, {Stage::kInitializing, "Initializing"}, {Stage::kTracking, "Tracking"}, {Stage::kRelocalization, "Reloc"}};

    const std::unordered_map<mivins::TrackingQuality, std::string, EnumClassHash>
        kTrackingQualityName{{TrackingQuality::kInsufficient, "Insufficient"},
                             {TrackingQuality::kBad, "Bad"},
                             {TrackingQuality::kGood, "Good"}};

    const std::unordered_map<mivins::UpdateResult, std::string, EnumClassHash>
        kUpdateResultName{{UpdateResult::kDefault, "Default"}, {UpdateResult::kKeyframe, "KF"}, {UpdateResult::kFailure, "Failure"}};

    ChannelFrameBase::ChannelFrameBase(const BaseOptions &base_options, const ReprojectorOptions &reprojector_options,
                                       const DepthOptimizationOptions &depthfilter_options,
                                       const DetectorOptions &detector_options, const InitializationOptions &init_options,
                                       const FeatureTrackerOptions &tracker_options, const CameraBundle::Ptr &cameras) : options_(base_options), cams_(cameras), stage_(Stage::kPaused), set_reset_(false), set_start_(false), map_(new FrontendLocalMap), num_obs_last_(0), tracking_quality_(TrackingQuality::kInsufficient), relocalization_n_trials_(0)
    {
        // sanity checks
        CHECK_EQ(reprojector_options.cell_size, detector_options.cell_size);

        need_new_kf_ = std::bind(&ChannelFrameBase::NeedNewKf, this, std::placeholders::_1);

        if (options_.trace_statistics)
        {
            std::string path = getenv("HOME")+options_.trace_dir;
            // Initialize Performance Monitor
            g_permon.reset(new vk::PerformanceMonitor());
            g_permon->addTimer("photometric_undistort");
            g_permon->addTimer("pyramid_creation");
            g_permon->addTimer("sparse_img_align");
            g_permon->addTimer("reproject");
            //g_permon->addTimer("reproject_kfs");
            //g_permon->addTimer("reproject_candidates");
            //g_permon->addTimer("feature_align");
            g_permon->addTimer("pose_optimizer");
            g_permon->addTimer("point_optimizer");
            //g_permon->addTimer("local_ba");
            g_permon->addTimer("frontend_time");
            g_permon->addTimer("fe_time");
            g_permon->addTimer("be_time");
            g_permon->addLog("timestamp");
            g_permon->addLog("img_align_n_tracked");
            //g_permon->addLog("repr_n_matches_local_map");
            //g_permon->addLog("repr_n_trials_local_map");
            //g_permon->addLog("repr_n_matches_global_map");
            //g_permon->addLog("repr_n_trials_global_map");
            //g_permon->addLog("sfba_thresh");
            //g_permon->addLog("sfba_error_init");
            //g_permon->addLog("sfba_error_final");
            //g_permon->addLog("sfba_n_edges_final");
            //g_permon->addLog("loba_n_erredges_init");
            //g_permon->addLog("loba_n_erredges_fin");
            //g_permon->addLog("loba_err_init");
            //g_permon->addLog("loba_err_fin");
            //g_permon->addLog("n_candidates");
            g_permon->addLog("dropout");
            g_permon->init("trace_frontend", path);
        }
        // init modules
        reprojectors_.reserve(cams_->getNumCameras());
        for (size_t camera_idx = 0; camera_idx < cams_->getNumCameras(); ++camera_idx)
        {
            reprojectors_.emplace_back(new Reprojector(reprojector_options, camera_idx));
        }
        SparseImgAlignOptions img_align_options;
        // img_align_options.max_level = options_.img_align_max_level;
        // img_align_options.min_level = options_.img_align_min_level;
        // img_align_options.robustification = options_.img_align_robustification;
        // img_align_options.use_distortion_jacobian = options_.img_align_use_distortion_jacobian;
        // img_align_options.estimate_illumination_gain = options_.img_align_est_illumination_gain;
        // img_align_options.estimate_illumination_offset = options_.img_align_est_illumination_offset;
        // img_align_options.estimate_ab = options_.img_align_est_ab;
        img_align_options.img_align_max_level = options_.img_align_max_level;
        img_align_options.img_align_min_level = options_.img_align_min_level;
        img_align_options.img_align_robustification = options_.img_align_robustification;
        img_align_options.img_align_use_distortion_jacobian = options_.img_align_use_distortion_jacobian;
        img_align_options.img_align_estimate_alpha = options_.img_align_estimate_alpha;
        img_align_options.img_align_estimate_beta = options_.img_align_estimate_beta;
        img_align_options.img_align_estimate_ab = options_.img_align_estimate_ab;

        sparse_img_align_.reset(new SparseImgAlign(SparseImgAlign::GetDefaultSolverOptions(), img_align_options));
        pose_optimizer_.reset(new PoseOptimizer(PoseOptimizer::getDefaultSolverOptions()));
        if (options_.poseoptim_using_unit_sphere)
            pose_optimizer_->SetErrorType(PoseOptimizer::ErrorType::kBearingVectorDiff);

        // DEBUG ***
        //pose_optimizer_->InitTracing(options_.trace_dir);
        DetectorOptions detector_options2 = detector_options;
        //detector_options2.detector_type = DetectorType::kGridGrad;

        depth_filter_.reset(new DepthOptimization(depthfilter_options, detector_options2, cams_));
        initializer_ = initialization_utils::makeInitializer(init_options, tracker_options, detector_options, cams_);
        overlap_kfs_.resize(cams_->getNumCameras());
        // VINS CODE >>>>>>>
        // if(options_.use_vins_backend)
        // {
        //   estimator = new Estimator();
        //   estimator->setIMUHandler(imu_handler_);
        //   estimator->loadParameters(options_.vins_config);
        //   estimator->setParameter();
        // }
        init_sucess = false;
        last_margedkf_t =0;

        // VINS CODE <<<<<<<
        VLOG(1) << "SVO initialized";
    }

    void ChannelFrameBase::savefile()
    {
        //std::cout<<"ChannelFrameBase::savefile-------"<<std::endl;
        LOG_DEBUG_STREAM("ChannelFrameBase::savefile-------");
        // std::string path = getenv("HOME");
        //savetwb.open(path + "/savelog/stamped_traj_estimate.txt");
        //savetwb.open("/SDCARD/miloc/maps/default/visual/trajectory.txt");// for map & miloc

        //savekf.open(path + "/savelog/kf_stamped_traj_estimate.txt");
        //savetkf.open("/SDCARD/miloc/maps/default/visual/kf_trajectory.txt");/
        std::cout<<"save_pose_path="<<options_.save_pose_path <<std::endl;
        std::cout<<"save_kf_pose_path="<<options_.save_kf_pose_path <<std::endl;
        remove(options_.save_pose_path.c_str());
        remove(options_.save_kf_pose_path.c_str());
        savetwb.open(options_.save_pose_path);
        savekf.open(options_.save_kf_pose_path);
    }

    void ChannelFrameBase::finish_savefile()
    {
        if(options_.backend_opt)
        {
            for (int i = 0; i <= window_size_; ++i)
            {
                FrameBundlePtr frame_bundle = vins_backend_->getFrameBundle(i);
                if(!frame_bundle)
                    continue;
                Transformation imu_pose = frame_bundle->Get_T_W_B();
                std::vector<double> pose = Twb2Twbaselink(imu_pose);
                savekf.precision(16);
                savekf << frame_bundle->GetMinTimestampSeconds() << " "
                   << pose[0] << " " << pose[1] << " " << pose[2] << " "
                   << pose[3] << " " << pose[4] << " " << pose[5] << " " << pose[6] << std::endl;
            }
            
        }
        savetwb.close();
        savekf.close();

    }
    

    ChannelFrameBase::~ChannelFrameBase()
    {
        //delete estimator;
        // need_new_kf_
        // VLOG(1) << "SVO destructor invoked";
    }

    //------------------------------------------------------------------------------
    bool ChannelFrameBase::AddImageBundle(const std::vector<cv::Mat> &imgs, const uint64_t timestamp)
    {
        // TicToc t_total;
        if (last_frames_)
        {
            // check if the timestamp is valid
            if (last_frames_->GetMinTimestampNanoseconds() >= static_cast<int64_t>(timestamp))
            {
                VLOG(4) << "Dropping frame: timestamp older than last frame of id " << last_frames_->GetBundleId();
                LOG_WARN_STREAM("Dropping frame: timestamp older than last frame.");
                return false;
            }
        }
        else
        {
            // at first iteration initialize tracing if enabled
            //if (options_.trace_statistics)
            //  bundle_adjustment_->setPerformanceMonitor(options_.trace_dir);
        }
        if (options_.trace_statistics)
        {
            SVO_START_TIMER("pyramid_creation");
        }
        std::vector<FramePtr> frames;
        // TicToc t_pyramid;
        if (GetType() == 3)
        {
            if (imgs.size() != 4)
            {
                std::cout << " IN[ChannelFrameBase::AddImageBundle] wrong size.!!!" << std::endl;
                return false;
            }
            //std::cout << "IN[ChannelFrameBase::AddImageBundle] type 3----------------------------" << std::endl;
            // {
            //   for (int m = 0; m < 10; m++)//imgs[3].rows
            //   {
            //     for (int n=0; n < imgs[3].cols; n++)
            //     {
            //       cout<<imgs[3].at<unsigned short>(m,n)<<" ";
            //     }
            //     cout<<std::endl;
            //   }
            // }
            frames.push_back(
                std::make_shared<Frame>(cams_->getCameraShared(0),
                                        imgs[0].clone(),
                                        imgs[3].clone(),
                                        timestamp,
                                        options_.img_align_max_level + 1, 1.0));
            frames.back()->Set_T_cam_imu(cams_->get_T_C_B(0)); // imu 到camera标定信息
            frames.back()->SetNFrameIndex(0);                  //  keyframe id

            //LeftImage And RightImage
            //暂时设置
            for (size_t i = 1; i < imgs.size() - 1; ++i)
            {                                                                                                                                       //1  left ;//2  right
                frames.push_back(std::make_shared<Frame>(cams_->getCameraShared(i), imgs[i].clone(), timestamp, options_.img_align_max_level + 1)); //JK: TODO pushback中的构造 keyframe, data
                frames.back()->Set_T_cam_imu(cams_->get_T_C_B(i));                                                                                  // imu 到camera标定信息
                frames.back()->SetNFrameIndex(i);                                                                                                   //  keyframe id
            }

            //std::cout<<"test in....."<<frames[0]->img_pyr_[0].rows<<","<<frames[0]->original_color_image_.rows<<std::endl;
            // cv::imshow("front", frames[0]->img_pyr_[0]);
            // cv::imshow("left", frames[1]->img_pyr_[0]);
            // cv::imshow("right", frames[2]->img_pyr_[0]);
            // cv::waitKey(0);
        }
        else
        {
            CHECK_EQ(imgs.size(), cams_->getNumCameras());
            for (size_t i = 0; i < imgs.size(); ++i)
            {
                frames.push_back(
                    std::make_shared<Frame>(cams_->getCameraShared(i), imgs[i].clone(), timestamp, options_.img_align_max_level + 1));
                frames.back()->Set_T_cam_imu(cams_->get_T_C_B(i));
                frames.back()->SetNFrameIndex(i);
            }
        }
        
        FrameBundlePtr frame_bundle(new FrameBundle(frames));
        if (options_.trace_statistics)
        {
            SVO_STOP_TIMER("pyramid_creation");
        }
        // Process frame bundle.
        bool flag = AddFrameBundle(frame_bundle);

        return flag;
    }

    bool ChannelFrameBase::AddImageBundle(const std::vector<cv::Mat> &imgs, const std::map<int, cv::Mat> &depths,
                                          const uint64_t timestamp, std::vector<float> &exposure_times)
    {
        // TicToc t_total;
        if (last_frames_)
        {
            // check if the timestamp is valid
            if (last_frames_->GetMinTimestampNanoseconds() >= static_cast<int64_t>(timestamp))
            {
                VLOG(4) << "Dropping frame: timestamp older than last frame of id " << last_frames_->GetBundleId();
                LOG_WARN_STREAM("Dropping frame: timestamp older than last frame.");
                return false;
            }
        }
        std::vector<FramePtr> frames;
        //triple: front left right
        if (options_.trace_statistics)
        {
            SVO_START_TIMER("pyramid_creation");
        }
        for (size_t i = 0; i < imgs.size(); ++i)
        {
            cv::Mat depth;
            if (depths.find(i) != depths.end())
                depth = depths.at(i);

            float exposure_time = 1.0;
            cv::Mat img = imgs[i].clone();
            if (exposure_times.size() > i)
            {
                //std::cout << "photometric process....." << std::endl;
                exposure_time = exposure_times[i];
            }
            if (options_.use_photometric_calibration && cams_->getCameraShared(i)->bgamma_ && cams_->getCameraShared(i)->bvignette_)
            {
                if (options_.trace_statistics)
                {
                    SVO_START_TIMER("photometric_undistort");
                }
                //cv::imshow("ori",img);
                //cv::waitKey(5);
                cams_->photometricUndistorter(i, img);
                //cv::imshow("undis",img);
                //cv::waitKey(5);
                if (options_.trace_statistics)
                {
                    SVO_STOP_TIMER("photometric_undistort");
                }
            }
            frames.push_back(std::make_shared<Frame>(cams_->getCameraShared(i),
                                                     img,
                                                     depth,
                                                     timestamp,
                                                     options_.img_align_max_level + 1,
                                                     exposure_time));

            frames.back()->Set_T_cam_imu(cams_->get_T_C_B(i));
            frames.back()->SetNFrameIndex(i);
        }
        if (options_.trace_statistics)
        {
            SVO_STOP_TIMER("pyramid_creation");
        }
        FrameBundlePtr frame_bundle(new FrameBundle(frames));
        bool flag = AddFrameBundle(frame_bundle);
        
        return flag;
    }

    bool ChannelFrameBase::AddImageBundle(const std::vector<cv::Mat> &imgs, std::vector<float> &exposure_times,
                                          const uint64_t timestamp)
    {
        // TicToc t_total;
        if (last_frames_)
        {
            // check if the timestamp is valid
            if (last_frames_->GetMinTimestampNanoseconds() >= static_cast<int64_t>(timestamp))
            {
                VLOG(4) << "Dropping frame: timestamp older than last frame of id " << last_frames_->GetBundleId();
                LOG_WARN_STREAM("Dropping frame: timestamp older than last frame.");
                return false;
            }
        }
        else
        {
            // at first iteration initialize tracing if enabled
            //if (options_.trace_statistics)
            //  bundle_adjustment_->setPerformanceMonitor(options_.trace_dir);
        }
        if (options_.trace_statistics)
        {
            SVO_START_TIMER("pyramid_creation");
        }

        std::vector<FramePtr> frames;
        // TicToc t_pyramid;
        //rgbdfisheye_type = 3
        if (GetType() == 3)
        {
            if (imgs.size() != 4)
            {
                std::cout << " IN[ChannelFrameBase::AddImageBundle] wrong size.!!!" << std::endl;
                return false;
            }
            frames.push_back(
                std::make_shared<Frame>(cams_->getCameraShared(0),
                                        imgs[1].clone(),
                                        imgs[0].clone(),
                                        timestamp,
                                        options_.img_align_max_level + 1, 1.0));
            frames.back()->Set_T_cam_imu(cams_->get_T_C_B(0)); // imu 到camera标定信息
            frames.back()->SetNFrameIndex(0);                  //  keyframe id

            //LeftImage And RightImage
            //暂时设置
            for (size_t i = 1; i < imgs.size() - 1; ++i)
            {                                                                                                                                       //1  left ;//2  right
                frames.push_back(std::make_shared<Frame>(cams_->getCameraShared(i), imgs[i].clone(), timestamp, options_.img_align_max_level + 1)); //JK: TODO pushback中的构造 keyframe, data
                frames.back()->Set_T_cam_imu(cams_->get_T_C_B(i));                                                                                  // imu 到camera标定信息
                frames.back()->SetNFrameIndex(i);                                                                                                   //  keyframe id
            }
        }
        else
        {
            CHECK_EQ(imgs.size(), cams_->getNumCameras());
            for (size_t i = 0; i < imgs.size(); ++i)
            {
                //std::cout << ".......exposure_time=" << exposure_times[i] << std::endl;
                frames.push_back(
                    std::make_shared<Frame>(cams_->getCameraShared(i), imgs[i].clone(), timestamp, exposure_times[i], options_.img_align_max_level + 1));
                frames.back()->Set_T_cam_imu(cams_->get_T_C_B(i));
                frames.back()->SetNFrameIndex(i);
            }
        }

        FrameBundlePtr frame_bundle(new FrameBundle(frames));
        if (options_.trace_statistics)
        {
            SVO_STOP_TIMER("pyramid_creation");
        }
        // Process frame bundle.
        bool flag = AddFrameBundle(frame_bundle);
        

        return flag;
    }

    //------------------------------------------------------------------------------
    bool ChannelFrameBase::AddFrameBundle(const FrameBundlePtr &frame_bundle)
    {
        vk::Timer tim;
        if (options_.trace_statistics)
        {
            SVO_START_TIMER("fe_time");
        }

        VLOG(40) << "New Frame Bundle received: " << frame_bundle->GetBundleId();
        //CHECK_EQ(frame_bundle->size(), cams_->numCameras());
        //std::cout << "...............New Frame Bundle received: " << frame_bundle->GetBundleId() << std::endl;
        LOG_DEBUG_STREAM("...............New Frame Bundle received: " << frame_bundle->GetBundleId());
        // ---------------------------------------------------------------------------AddFrameBundle
        // Prepare processing.

        if (set_start_)
        {
            // Temporary copy rotation prior. TODO(cfo): fix this.
            Quaternion R_imu_world = R_imu_world_;
            Eigen::Vector3d t_imu_world = t_imu_world_;
            bool have_rotation_prior = have_rotation_prior_;
            bool have_position_prior = have_position_prior_;
            ResetAll();
            R_imu_world_ = R_imu_world;
            t_imu_world_ = t_imu_world;
            have_rotation_prior_ = have_rotation_prior;
            have_position_prior_ = have_position_prior;
            SetInitialPose(frame_bundle);
            stage_ = Stage::kInitializing;
            //std::cout << "set initial pose -----------------------------" << std::endl;
        }

        if(is_abnormal_)
        {
          ResetAll();
          stage_ = Stage::kInitializing;
          frame_bundle->Set_T_W_B(abnormal_pose_);
        }

        if(stage_ == Stage::kInitializing && abnormal_cnt_ > 0)
            frame_bundle->Set_T_W_B(abnormal_pose_);
          
        if (stage_ == Stage::kPaused)
        {
            return false;
        }

        if (options_.trace_statistics)
        {
            SVO_LOG("timestamp", frame_bundle->at(0)->GetTimestampNSec());
            SVO_START_TIMER("frontend_time");
            //if (bundle_adjustment_)
            //{
            //  bundle_adjustment_->startTimer(frame_bundle->GetBundleId());
            //}
        }
        timer_.start();

        // ---------------------------------------------------------------------------
        // Add to pipeline.
        new_frames_ = frame_bundle;
        ++frame_counter_;

        if (options_.backend_opt)
            vins_backend_->updateLatestStates(new_frames_, last_frames_);

#ifdef SVO_GLOBAL_MAP
        if (global_map_ && imu_handler_ && last_frames_)
        {
            ImuMeasurements imu_meas_since_last;
            const double cur_t_sec = new_frames_->GetMinTimestampSeconds();
            const double last_t_sec = last_frames_->GetMinTimestampSeconds();
            if (imu_handler_->WaitTill(cur_t_sec))
            {
                imu_handler_->GetMeasurements(last_t_sec, cur_t_sec, false,
                                              imu_meas_since_last);
            }
            global_map_->accumulateIMUMeasurements(imu_meas_since_last);
        }
#endif

        // for scale check
        double svo_dist_first_two_kfs = -1;
        double opt_dist_first_two_kfs = -1;
        bool backend_scale_stable = false;
        // if we have bundle adjustment running in parallel thread, check if it has
        // computed a new map. in this case, we replace our map with the latest estimate
        // for the CeresBackend this value should always be obtainable

        //SetMotionPrior(new_frames_);
        
        SetMotionPrior(new_frames_);
        if (options_.backend_opt)
        {
            vins_backend_->inputImu(new_frames_);
            vins_backend_->inputOdom(new_frames_);
        }
        // Perform tracking.
        update_res_ = ProcessFrameBundle();
        if (!new_frames_)
            return false;
        //savekf<< new_frames_->GetMinTimestampSeconds()<<":"<<new_frames_->at(0)->IsKeyframe()<<std::endl;
        // We start the backend first, since it is the most time crirical
        //   if (bundle_adjustment_)
        //   {
        // #ifdef SVO_LOOP_CLOSING
        //     // if we have loop closing module, see whether there is any correction we can do
        //     if (lc_)
        //     {
        //       std::lock_guard<std::mutex> lock(lc_->lc_info_lock_);
        //       if (lc_->hasCorrectionInfo())
        //       {
        //         Transformation w_T_correction;
        //         lc_->consumeOldestCorrection(&w_T_correction);
        //         bundle_adjustment_->setCorrectionInWorld(w_T_correction);
        //         SetRecovery(false);
        //       }
        //     }
        // #endif
        //     VLOG(40) << "Call bundle adjustment.";
        //     //bundle_adjustment_->bundleAdjustment(new_frames_);
        //   }
        if (options_.trace_statistics)
        {
            SVO_STOP_TIMER("fe_time");
            SVO_START_TIMER("be_time");
        }
        double fe_ts = tim.stop() * 1000.0f;

        tim.start();
        if (stage_ == Stage::kTracking)
        {
            if (!init_sucess)
            {
                init_sucess = true;
            }
            else
            {
                // if(new_frames_->at(0)->IsKeyframe())
                // {
                //   if(options_.use_vins_backend)
                //   {
                //     estimator->svoProcessFrame(map_, new_frames_);
                //     if(!options_.vins_backend_multi_thread)
                //     {
                //       estimator->svoLoadMap(new_frames_, map_);
                //     }
                //   }
                // }
                // if(options_.use_vins_backend && options_.vins_backend_multi_thread)
                //   estimator->svoLoadMap(new_frames_, map_);
                //std::cout << "**************************************************options_.backend_opt" << options_.backend_opt << std::endl;
                
                if(update_res_ == UpdateResult::kKeyframe)
                    new_frames_->SetKeyframe();
                    
                if (options_.backend_opt)
                {
                    vins_backend_->addFrameBundle(new_frames_);
                }
            }
        }

        if (options_.trace_statistics)
        {
            SVO_STOP_TIMER("be_time");
        }
        double be_ts = tim.stop() * 1000.0f;
        std::cout << new_frames_->IsKeyframe() << "; "
                  << "fe ts: " << fe_ts << "; "
                  << "be ts: " << be_ts << "; "
                  << "all ts: " << (fe_ts + be_ts) << "\n";


        // Information exchange between loop closing and global map
        MatchedPointsInfo point3d_match_info;
#ifdef SVO_LOOP_CLOSING
#ifdef SVO_GLOBAL_MAP
        if (lc_)
        {
            std::lock_guard<std::mutex> lock(lc_->lc_info_lock_);
            lc_->consumePointMatchInfo(&point3d_match_info);
        }
        BundleIdToIMUPose Twb_map;
        if (global_map_)
        {
            global_map_->addMatchingPointInfo(point3d_match_info);
            global_map_->getBundleIDIMUPosesMap(&Twb_map);
        }
        if (lc_ && lc_->useExternalMap())
        {
            lc_->updateKeyframePoses(Twb_map);
        }
#endif
#endif

        // Add keyframe to loop closing and the global map
        int frame_flag = 0;
        if (update_res_ == UpdateResult::kKeyframe)
        {
            // Set flag in bundle. Before we only set each frame individually.
            frame_flag = 1;
            new_frames_->SetKeyframe();
            last_kf_time_sec_ = new_frames_->at(0)->GetTimestampSec();
            FramePtr last_rm_kf = nullptr;
            map_->GetLastRemovedKF(&last_rm_kf);
#ifdef SVO_LOOP_CLOSING
            if (lc_ && last_rm_kf)
            {
                // if we removed any keyframe,
                // update the keyframe information and trigger pose graph optimization if needed
                lc_->updateKeyframe(last_rm_kf);
            }
#endif
#ifdef SVO_GLOBAL_MAP
            if (global_map_)
            {
                global_map_->startNewAccumulation(new_frames_->at(0)->GetFrameId());
                // it can happen that no keyframe is removed yet
                if (last_rm_kf)
                {
                    if (IsBackendValid() && backend_scale_initialized_)
                    {
                        global_map_->AddKeyframe(last_rm_kf);
                    }
                    if (!global_map_has_initial_ba_)
                    {
                        global_map_has_initial_ba_ = global_map_->hasInitialBA();
                    }
                }
            }
#endif
        }

        // #ifdef SVO_LOOP_CLOSING
        //   // we add the previous frame if it is a keyframe
        //   // this is to wait till it is optimized
        //   if (lc_ && last_frames_ && last_frames_->IsKeyframe())
        //   {
        //     if (!bundle_adjustment_ ||
        //         (backend_scale_stable && backend_scale_initialized_))
        //     {
        //       lc_->addFrameToPR(last_frames_);
        //     }
        //   }
        // #endif
        // ---------------------------------------------------------------------------
        // Finish pipeline.

        if (last_frames_)
        {
            // Set translation motion prior for next frame.
            t_lastimu_newimu_ = new_frames_->at(0)->T_imu_world().GetRotation().Rotate(
                new_frames_->at(0)->ImuPos() - last_frames_->at(0)->ImuPos());
        }

        // Statistics.
        num_obs_last_ = new_frames_->NumTrackedFeatures() +
                        new_frames_->NumFixedLandmarks();
        if (stage_ == Stage::kTracking)
        {
            if (IsInRecovery())
            {
                CHECK_GT(new_frames_->GetMinTimestampSeconds(),
                         last_good_tracking_time_sec_);
                if ((new_frames_->GetMinTimestampSeconds() - last_good_tracking_time_sec_) < options_.global_map_lc_timeout_sec_)
                {
                    SetRecovery(false);
                    last_good_tracking_time_sec_ = new_frames_->GetMinTimestampSeconds();
                }
            }
            else
            {
                last_good_tracking_time_sec_ = new_frames_->GetMinTimestampSeconds();
            }
        }

        // Try relocalizing if tracking failed.
        if (update_res_ == UpdateResult::kFailure)
        {
            if (options_.backend_opt)
                vins_backend_->reset();
            VLOG(2) << "Tracking failed: RELOCALIZE.";
            CHECK(stage_ == Stage::kTracking || stage_ == Stage::kInitializing || stage_ == Stage::kRelocalization);

            // Let's try to relocalize with respect to the last keyframe:
            reloc_keyframe_ = map_->GetLastKeyframe(); //前  按照svo规则:在添加关键帧之前返回kFailure
            CHECK_NOTNULL(reloc_keyframe_.get());

            bool bUseRlocSvo = false; //TODO param
            if (bUseRlocSvo)
            {
                if (!map_->GetKeyframeById(map_->last_added_kf_id_ - 2)->depth_image_.empty())
                { //前
                    std::vector<FramePtr> frames;
                    frames.push_back(map_->GetKeyframeById(map_->last_added_kf_id_ - 2)); //前
                    frames.push_back(map_->GetKeyframeById(map_->last_added_kf_id_ - 1)); //左
                    frames.push_back(map_->GetKeyframeById(map_->last_added_kf_id_));     //右

                    FrameBundlePtr frame_bundle_loc_(new FrameBundle(frames));
                    reloc_keyframe_bundle_ = frame_bundle_loc_;
                }
                else
                {
                    std::cout << "----------------------------error-------------------------------not front in reloc  ------------" << std::endl;
                }
            }
            // Reset pose to previous frame to avoid crazy jumps.
            if (stage_ == Stage::kTracking && last_frames_)
            {
                for (size_t i = 0; i < last_frames_->size(); ++i)
                    new_frames_->at(i)->T_f_w_ = last_frames_->at(i)->T_f_w_;
            }

            // Reset if we tried many times unsuccessfully to relocalize.
            if (stage_ == Stage::kRelocalization &&
                relocalization_n_trials_ >= options_.relocalization_max_trials)
            {
                VLOG(2) << "Relocalization failed "
                        << options_.relocalization_max_trials << " times: RESET.";
                set_reset_ = true;
                backend_reinit_ = true;
            }

            // Set stage.
            stage_ = Stage::kRelocalization;
            LOG_DEBUG_STREAM("IN[AddFrameBundle]---------------------------------Stage::kRelocalization---------------------------------------!");
            //std::cout << "IN[AddFrameBundle]---------------------------------Stage::kRelocalization---------------------------------------!" << std::endl;
            tracking_quality_ = TrackingQuality::kInsufficient;
        }
        
        // Calc vel and omg for constant speed model
        calcVelAndOmg();

        // Set last frame.
        last_frames_ = new_frames_;
        new_frames_.reset();

        // Reset if we should.
        if (set_reset_)
        {
            ResetVisionFrontendCommon();
            ResetBackend();
        }

        // Reset rotation prior.
        have_rotation_prior_ = false;
        R_imulast_world_ = R_imu_world_;

        // Reset motion prior
        have_motion_prior_ = false;
        T_newimu_lastimu_prior_.setIdentity();

        // tracing
        if (options_.trace_statistics)
        {
            SVO_LOG("dropout", static_cast<int>(update_res_));
            SVO_STOP_TIMER("frontend_time");
            g_permon->writeToFile();
        }
        // Eigen::Quaterniond q = last_frames_->Get_T_W_B().GetRotation().ToImplementation();
        // Eigen::Vector3d p(last_frames_->Get_T_W_B().GetPosition());
        // savetwb.precision(16);
        // savetwb << last_frames_->GetMinTimestampSeconds() << " "
        //         << p.x() << " " << p.y() << " " << p.z() << " "
        //         << q.x() << " " << q.y() << " " << q.z() << " " << q.w() 
        //         << std::endl;
        // Transformation cam_pose = last_frames_->at(0)->T_world_cam();
        // Eigen::Quaterniond q_rsleft_to_baselink(0.4497752,-0.545621, 0.545621, -0.4497752);
        // Eigen::Vector3d t_rsleft_to_baselink(0.275309,0.025,0.114282);
        // mivins::Transformation T_baselink_rsleft(Quaternion(q_rsleft_to_baselink), t_rsleft_to_baselink);
        // mivins::Transformation T_out=cam_pose*(T_baselink_rsleft.Inverse());
        // Eigen::Quaterniond q_out = T_out.GetRotation().ToImplementation();
        // Eigen::Vector3d p_out(T_out.GetPosition());
        // savetwb.precision(16);
        // savetwb << last_frames_->GetMinTimestampSeconds() << " "
        //         << p_out.x() << " " << p_out.y() << " " << p_out.z() << " "
        //         << q_out.x() << " " << q_out.y() << " " << q_out.z() << " " << q_out.w() << " "
        //         << frame_flag << std::endl;

        {
            Transformation imu_pose = last_frames_->Get_T_W_B();
            std::vector<double> pose = Twb2Twbaselink(imu_pose);
            savetwb.precision(16);
            savetwb << last_frames_->GetMinTimestampSeconds() << " "
                   << pose[0] << " " << pose[1] << " " << pose[2] << " "
                   << pose[3] << " " << pose[4] << " " << pose[5] << " " << pose[6] << " "
                   << frame_flag << std::endl;
        }


        if(options_.backend_opt)
        {
            FrameBundlePtr marged_KF;
            vins_backend_->getMargedKF(marged_KF);
            if(marged_KF)
            {
                if(last_margedkf_t != marged_KF->GetMinTimestampSeconds())
                {
                    Transformation T_w_b = marged_KF->Get_T_W_B();
                    std::vector<double> pose = Twb2Twbaselink(T_w_b);
                    savekf.precision(16);
                    savekf << last_frames_->GetMinTimestampSeconds() << " "
                        << pose[0] << " " << pose[1] << " " << pose[2] << " "
                        << pose[3] << " " << pose[4] << " " << pose[5] << " " << pose[6] << std::endl;

                    last_margedkf_t = marged_KF->GetMinTimestampSeconds();
                }

            }
        }

        return true;
    }

    std::vector<double> ChannelFrameBase::Twb2Twbaselink(const Transformation &T_w_b)
    {
        
        Transformation T_b_o;
        if(odom_handler_)
            T_b_o= odom_handler_->odom_calib_.T_B_O;
        else
            T_b_o = Transformation(Eigen::Vector3d::Zero(),
                  Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
        Transformation T_w_o = T_w_b * T_b_o;
        
        Eigen::Quaterniond q = T_w_o.GetRotation().ToImplementation();
        Eigen::Vector3d p = T_w_o.GetPosition();
        std::vector<double> pose = {p[0], p[1], p[2], q.x(), q.y(), q.z(), q.w()};
        return pose;
    }

    //------------------------------------------------------------------------------
    void ChannelFrameBase::SetRotationPrior(const Quaternion &R_imu_world)
    {
        VLOG(40) << "Set rotation prior.";
        R_imu_world_ = R_imu_world;
        have_rotation_prior_ = true;
    }

    void ChannelFrameBase::SetRotationIncrementPrior(const Quaternion &R_lastimu_newimu)
    {
        VLOG(40) << "Set rotation increment prior.";
        R_imu_world_ = R_lastimu_newimu.Inverse() * R_imulast_world_;
        have_rotation_prior_ = true;
    }

    void ChannelFrameBase::SetPositionPrior(const Eigen::Vector3d &t_imu_world)
    {
        VLOG(40) << "Set position prior.";
        t_imu_world_ = t_imu_world;
        have_position_prior_ = true;
    }

    //------------------------------------------------------------------------------
    void ChannelFrameBase::SetInitialPose(const FrameBundlePtr &frame_bundle) const
    {
        if (have_rotation_prior_)
        {
            VLOG(40) << "Set initial pose: With rotation prior";
            for (size_t i = 0; i < frame_bundle->size(); ++i)
            {
                frame_bundle->at(i)->T_f_w_ = cams_->get_T_C_B(i) * Transformation(R_imu_world_, Vector3d::Zero());
            }
        }
        else if (frame_bundle->imu_measurements_.cols() > 0)
        {
            VLOG(40) << "Set initial pose: Use inertial measurements in frame to get gravity.";
            const Vector3d g = frame_bundle->imu_measurements_.topRows<3>().rowwise().sum();
            const Vector3d z = g.normalized(); // imu measures positive-z when static
            // TODO: make sure z != -1,0,0
            Vector3d p(1, 0, 0);
            Vector3d p_alternative(0, 1, 0);
            if (std::fabs(z.dot(p)) > std::fabs(z.dot(p_alternative)))
                p = p_alternative;
            Vector3d y = z.cross(p); // make sure gravity is not in x direction
            y.normalize();
            const Vector3d x = y.cross(z);
            Matrix3d C_imu_world; // world unit vectors in imu coordinates
            C_imu_world.col(0) = x;
            C_imu_world.col(1) = y;
            C_imu_world.col(2) = z;
            Transformation T_imu_world(Quaternion(C_imu_world), Eigen::Vector3d::Zero());
            frame_bundle->Set_T_W_B(T_imu_world.Inverse());
            VLOG(3) << "Initial Rotation = " << std::endl
                    << C_imu_world.transpose() << std::endl;
        }
        else
        {
            VLOG(40) << "Set initial pose: set such that T_imu_world is identity.";
            for (size_t i = 0; i < frame_bundle->size(); ++i)
            {
                frame_bundle->at(i)->T_f_w_ = cams_->get_T_C_B(i) * T_world_imuinit.Inverse();
            }
        }
    }

    //------------------------------------------------------------------------------
    size_t ChannelFrameBase::SparseImageAlignment()
    {
        // optimize the pose of the new frame such that it matches the pose of the previous frame best
        // this will improve the relative transformation between the previous and the new frame
        // the result is the number of feature points which could be tracked
        // this is a hierarchical KLT solver
        VLOG(40) << "Sparse image alignment.";
        if (options_.trace_statistics)
        {
            SVO_START_TIMER("sparse_img_align");
        }
        sparse_img_align_->Reset();
        if (have_motion_prior_)
        {
            LOG_DEBUG_STREAM("Apply IMU Prior to Image align");
            double prior_trans = options_.img_align_prior_lambda_trans;
            if (map_->size() < 5)
                prior_trans = 0; // during the first few frames we don't want a prior (TODO)

            sparse_img_align_->SetWeightedPrior(T_newimu_lastimu_prior_, 0.0, 0.0,
                                                options_.img_align_prior_lambda_rot,
                                                prior_trans, 0.0, 0.0);
        }
        sparse_img_align_->SetMaxNumFeaturesToAlign(options_.img_align_max_num_features);
        size_t img_align_n_tracked = sparse_img_align_->Run(last_frames_, new_frames_);

        if (options_.trace_statistics)
        {
            SVO_STOP_TIMER("sparse_img_align");
            SVO_LOG("img_align_n_tracked", img_align_n_tracked);
        }
        VLOG(40) << "Sparse image alignment tracked " << img_align_n_tracked << " features.";
        return img_align_n_tracked;
    }

    //------------------------------------------------------------------------------
    size_t ChannelFrameBase::ProjectMapInFrame()
    {
        VLOG(40) << "Project map in frame.";
        if (options_.trace_statistics)
        {
            SVO_START_TIMER("reproject");
        }
        // compute overlap keyframes
        for (size_t camera_idx = 0; camera_idx < cams_->numCameras(); ++camera_idx)
        {
            ReprojectorPtr &cur_reprojector = reprojectors_.at(camera_idx);
            overlap_kfs_.at(camera_idx).clear();
            map_->GetClosestNKeyframesWithOverlap(
                new_frames_->at(camera_idx),
                cur_reprojector->options_.max_n_kfs,
                &overlap_kfs_.at(camera_idx));
#ifdef SVO_GLOBAL_MAP
            if (!IsInRecovery() &&
                cur_reprojector->options_.use_kfs_from_global_map && global_map_)
            {
                std::vector<FramePtr> overlap_kfs_global;
                global_map_->getOverlapKeyframesMaxN(
                    *new_frames_->at(camera_idx),
                    cur_reprojector->options_.max_n_global_kfs,
                    &overlap_kfs_global);
                overlap_kfs_.at(camera_idx).insert(overlap_kfs_.at(camera_idx).end(), std::make_move_iterator(overlap_kfs_global.begin()), std::make_move_iterator(overlap_kfs_global.end()));
            }
#endif
        }

        std::vector<std::vector<PointPtr>> trash_points;
        trash_points.resize(cams_->numCameras());
        if (options_.use_async_reprojectors && cams_->numCameras() > 1)
        {
            // start reprojection workers
            std::vector<std::future<void>> reprojector_workers;
            for (size_t camera_idx = 0; camera_idx < cams_->numCameras(); ++camera_idx)
            {
                auto func = std::bind(&Reprojector::ReprojectFrames, reprojectors_.at(camera_idx).get(),
                                      new_frames_->at(camera_idx), overlap_kfs_.at(camera_idx), trash_points.at(camera_idx));
                reprojector_workers.push_back(std::async(std::launch::async, func));
            }

            // make sure all of them are finished
            for (size_t i = 0; i < reprojector_workers.size(); ++i)
                reprojector_workers[i].get();
        }
        else
        {
            for (size_t camera_idx = 0; camera_idx < cams_->numCameras(); ++camera_idx)
            {
                reprojectors_.at(camera_idx)->ReprojectFrames(new_frames_->at(camera_idx), overlap_kfs_.at(camera_idx), trash_points.at(camera_idx));
            }
        }

        // Effectively clear the points that were discarded by the reprojectors
        for (auto point_vec : trash_points)
            for (auto point : point_vec)
                map_->SafeDeletePoint(point);

        // Count the total number of trials and matches for all reprojectors
        Reprojector::Statistics cumul_stats_;
        Reprojector::Statistics cumul_stats_global_map;
        for (const ReprojectorPtr &reprojector : reprojectors_)
        {
            cumul_stats_.n_matches += reprojector->stats_.n_matches;
            cumul_stats_.n_trials += reprojector->stats_.n_trials;
            cumul_stats_global_map.n_matches += reprojector->fixed_lm_stats_.n_matches;
            cumul_stats_global_map.n_trials += reprojector->fixed_lm_stats_.n_trials;
        }

        if (options_.trace_statistics)
        {
            SVO_STOP_TIMER("reproject");
            //SVO_LOG("repr_n_matches_local_map", cumul_stats_.n_matches);
            //SVO_LOG("repr_n_trials_local_map", cumul_stats_.n_trials);
            //SVO_LOG("repr_n_matches_global_map", cumul_stats_global_map.n_matches);
            //SVO_LOG("repr_n_trials_global_map", cumul_stats_global_map.n_trials);
        }
        VLOG(40) << "Reprojection:"
                 << "\t nPoints = " << cumul_stats_.n_trials << "\t\t nMatches = "
                 << cumul_stats_.n_matches;

        size_t n_total_ftrs = cumul_stats_.n_matches +
                              (cumul_stats_global_map.n_matches <= 10 ? 0 : cumul_stats_global_map.n_matches);

        if (n_total_ftrs < options_.quality_min_fts)
        {
            LOG_WARN_STREAM_THROTTLE(1.0, "Not enough matched features: " +
                                              std::to_string(n_total_ftrs));
        }

        return n_total_ftrs;
    }

    //------------------------------------------------------------------------------
    size_t ChannelFrameBase::OptimizePose()
    {
        // pose optimization
        // optimize the pose of the frame in such a way, that the projection of all feature world coordinates
        // is not far off the position of the feature points within the frame. The optimization is done for all points
        // in the same time, hence optimizing frame pose.
        if (options_.trace_statistics)
        {
            SVO_START_TIMER("pose_optimizer");
        }

        pose_optimizer_->Reset();
        if (have_motion_prior_)
        {
            VLOG(40) << "Apply prior to pose optimization";
            pose_optimizer_->SetRotationPrior(new_frames_->Get_T_W_B().GetRotation().Inverse(),
                                              options_.poseoptim_prior_lambda);
        }
        size_t sfba_n_edges_final = pose_optimizer_->run(new_frames_, options_.poseoptim_thresh);

        if (options_.trace_statistics)
        {
            //SVO_LOG("sfba_error_init", pose_optimizer_->stats_.reproj_error_before);
            //SVO_LOG("sfba_error_final", pose_optimizer_->stats_.reproj_error_after);
            //SVO_LOG("sfba_n_edges_final", sfba_n_edges_final);
            SVO_STOP_TIMER("pose_optimizer");
        }
        LOG_DEBUG_STREAM(
            "PoseOptimizer:"
            << "\t ErrInit = " << pose_optimizer_->stats_.reproj_error_before << "\t ErrFin = " << pose_optimizer_->stats_.reproj_error_after << "\t nObs = " << sfba_n_edges_final);
        return sfba_n_edges_final;
    }

    //------------------------------------------------------------------------------
    void ChannelFrameBase::OptimizeStructure(const FrameBundle::Ptr &frames, int max_n_pts, int max_iter)
    {
        VLOG(40) << "Optimize structure.";
        // some feature points will be optimized w.r.t keyframes they were observed
        // in the way that their projection error into all other keyframes is minimzed

        if (max_n_pts == 0)
            return; // don't return if max_n_pts == -1, this means we optimize ALL points

        if (options_.trace_statistics)
        {
            SVO_START_TIMER("point_optimizer");
        }
        for (const FramePtr &frame : frames->frames_)
        {
            bool optimize_on_sphere = false;
            if (frame->cam()->GetType() == Camera::Type::kOmni || frame->cam()->GetType() == Camera::Type::kMei)
                optimize_on_sphere = true;
            std::deque<PointPtr> pts;
            for (size_t i = 0; i < frame->num_features_; ++i)
            {
                if (frame->landmark_vec_[i] == nullptr || isEdgelet(frame->type_vec_[i]))
                    continue;
                pts.push_back((frame->landmark_vec_[i]));
            }
            auto it_end = pts.end();
            if (max_n_pts > 0)
            {
                max_n_pts = std::min(static_cast<size_t>(max_n_pts), pts.size());
                std::nth_element(pts.begin(), pts.begin() + max_n_pts, pts.end(), [](const PointPtr &lhs, const PointPtr &rhs)
                                 {
                                     // we favour points that have not been optimized in a while
                                     return (lhs->last_structure_optim_ < rhs->last_structure_optim_);
                                 });
                it_end = pts.begin() + max_n_pts;
            }
            for (const PointPtr &point : pts)
            {
                point->Optimize(max_iter, optimize_on_sphere);
                point->last_structure_optim_ = frame->GetFrameId();
            }
        }
        if (options_.trace_statistics)
        {
            SVO_STOP_TIMER("point_optimizer");
        }
    }
    bool ChannelFrameBase::CreateLandmarkWithDepth(
        const FramePtr &frame_ref, const FramePtr &frame_cur,
        const int id_ref, const int id_cur, PointPtr &point)
    {
        if (frame_cur->depth_image_.empty() || frame_ref->depth_image_.empty())
            return false;

        Eigen::Vector3d bv_ref = frame_ref->f_vec_.col(id_ref);
        Eigen::Vector3d bv_cur = frame_cur->f_vec_.col(id_cur);

        Eigen::Vector3d norm_px_ref = bv_ref / bv_ref.z();
        Eigen::Vector3d norm_px_cur = bv_cur / bv_cur.z();

        Eigen::Vector2d px_ref = frame_ref->px_vec_.col(id_ref);
        Eigen::Vector2d px_cur = frame_cur->px_vec_.col(id_cur);

        int x_ref = std::round(px_ref.x()), y_ref = std::round(px_ref.y());
        int x_cur = std::round(px_cur.x()), y_cur = std::round(px_cur.y());

        float depth_ref = frame_ref->GetValidDepthFromImage(y_ref, x_ref);
        float depth_cur = frame_cur->GetValidDepthFromImage(y_ref, x_ref);

        int pixel_error = 1.0f; //TODO param
        Eigen::VectorXd intrinsic = frame_ref->cam_->getIntrinsicParameters();
        float focal_length = std::sqrt(intrinsic(0) * intrinsic(1));
        ; //std::sqrt(intrinsic(0) * intrinsic(1));  //TODO param
        float residual_thresh = pixel_error / focal_length;

        if (depth_ref >= depth_img_min_ && depth_ref <= depth_img_max_)
        {
            Transformation T_cur_ref = frame_cur->T_cam_world() * frame_ref->T_world_cam();
            Eigen::Vector3d pts_cam_ref = norm_px_ref * depth_ref;
            Eigen::Vector3d pts_cam_cur = T_cur_ref * pts_cam_ref;
            Eigen::Vector2d residual = norm_px_cur.head<2>() - pts_cam_cur.head<2>() / pts_cam_cur.z();

            if (residual.norm() < residual_thresh)
            {
                // std::cout << "ref ok\n";
                Position xyz_world = frame_ref->T_world_cam() * pts_cam_ref;
                point = std::make_shared<Point>(xyz_world);
                frame_ref->landmark_vec_[id_ref] = point;
                frame_ref->track_id_vec_[id_ref] = point->Id();
                point->AddObservation(frame_ref, id_ref);
                point->SetConstant();
                return true;
            }
        }

        if (depth_cur >= depth_img_min_ && depth_cur <= depth_img_max_) //TODO param
        {
            Transformation T_ref_cur = frame_ref->T_cam_world() * frame_cur->T_world_cam();
            Eigen::Vector3d pts_cam_cur = norm_px_cur * depth_cur;
            Eigen::Vector3d pts_cam_ref = T_ref_cur * pts_cam_cur;
            Eigen::Vector2d residual = norm_px_ref.head<2>() - pts_cam_ref.head<2>() / pts_cam_ref.z();

            // std::cout << "ref: " << residual.norm() << "; "
            //           << "thresh: "<< residual_thresh << "\n";

            if (residual.norm() < residual_thresh)
            {
                // std::cout << "cur ok\n";
                Position xyz_world = frame_ref->T_world_cam() * pts_cam_ref;
                point = std::make_shared<Point>(xyz_world);
                frame_ref->landmark_vec_[id_ref] = point;
                frame_ref->track_id_vec_[id_ref] = point->Id();
                point->AddObservation(frame_ref, id_ref);
                point->SetConstant();
                return true;
            }
        }

        return false;
    }
    //------------------------------------------------------------------------------
    void ChannelFrameBase::UpgradeSeedsToFeatures(const FramePtr &frame)
    {
        VLOG(40) << "Upgrade seeds to features";
        size_t update_count = 0;
        size_t unconverged_cnt = 0;
        for (size_t i = 0; i < frame->num_features_; ++i)
        {
            if (frame->landmark_vec_[i])
            {
                const FeatureType &type = frame->type_vec_[i];
                if (type == FeatureType::kCorner || type == FeatureType::kEdgelet ||
                    type == FeatureType::kMapPoint)
                {
                    frame->landmark_vec_[i]->AddObservation(frame, i);
                }
                else
                {
                    CHECK(isFixedLandmark(type));
                    frame->landmark_vec_[i]->AddObservation(frame, i);
                }
            }
            else if (frame->seed_ref_vec_[i].keyframe)
            {
                if (isUnconvergedSeed(frame->type_vec_[i]))
                {
                    unconverged_cnt++;
                }
                SeedRef &ref = frame->seed_ref_vec_[i];

                // In multi-camera case, it might be that we already created a 3d-point
                // for this seed previously when processing another frame from the bundle.
                PointPtr point = ref.keyframe->landmark_vec_[ref.seed_id];
                if (point == nullptr)
                {
                    if (!CreateLandmarkWithDepth(ref.keyframe, frame, ref.seed_id, i, point))
                    {
                        // That's not the case. Therefore, create a new 3d point.
                        Position xyz_world =
                            ref.keyframe->T_world_cam() *
                            ref.keyframe->GetSeedPosInFrame(ref.seed_id);
                        point = std::make_shared<Point>(xyz_world);
                        ref.keyframe->landmark_vec_[ref.seed_id] = point;
                        ref.keyframe->track_id_vec_[ref.seed_id] = point->Id();
                        point->AddObservation(ref.keyframe, ref.seed_id);
                    }
                }

                // add reference to current frame.
                frame->landmark_vec_[i] = point;
                frame->track_id_vec_[i] = point->Id();
                point->AddObservation(frame, i);
                if (isCorner(ref.keyframe->type_vec_[ref.seed_id]))
                {
                    ref.keyframe->type_vec_[ref.seed_id] = FeatureType::kCorner;
                    frame->type_vec_[i] = FeatureType::kCorner;
                }
                else if (isMapPoint(ref.keyframe->type_vec_[ref.seed_id]))
                {
                    ref.keyframe->type_vec_[ref.seed_id] = FeatureType::kMapPoint;
                    frame->type_vec_[i] = FeatureType::kMapPoint;
                }
                else if (isEdgelet(ref.keyframe->type_vec_[ref.seed_id]))
                {
                    ref.keyframe->type_vec_[ref.seed_id] = FeatureType::kEdgelet;
                    frame->type_vec_[i] = FeatureType::kEdgelet;

                    // Update the edgelet direction.
                    double angle = feature_detector_utils::GetAngleAtPixelUsingHistogram(
                        frame->img_pyr_[frame->level_vec_[i]],
                        (frame->px_vec_.col(i) / (1 << frame->level_vec_[i])).cast<int>(),
                        4u);
                    frame->grad_vec_.col(i) = GradientVector(std::cos(angle),
                                                             std::sin(angle));
                }
                else
                {
                    CHECK(false) << "Seed-Type not known";
                }
                ++update_count;
            }

            // when using the feature-wrapper, we might copy some old references?
            frame->seed_ref_vec_[i].keyframe.reset();
            frame->seed_ref_vec_[i].seed_id = -1;
        }
        VLOG(5) << "NEW KEYFRAME: Updated "
                << update_count << " seeds to features in reference frame, "
                << "including " << unconverged_cnt << " unconverged points.\n";
        const double ratio = (1.0 * unconverged_cnt) / update_count;
        if (ratio > 0.2)
        {
            LOG(WARNING) << ratio * 100 << "% updated seeds are unconverged.";
        }
    }

    //------------------------------------------------------------------------------
    void ChannelFrameBase::ResetVisionFrontendCommon()
    {
        stage_ = Stage::kPaused;
        tracking_quality_ = TrackingQuality::kInsufficient;
        first_img_ = true;
        set_reset_ = false;
        set_start_ = false;
        frontend_abnormal_ = false;
        num_obs_last_ = 0;
        reloc_keyframe_.reset();
        reloc_keyframe_bundle_.reset();
        relocalization_n_trials_ = 0;
        t_lastimu_newimu_ = Vector3d::Zero();
        have_motion_prior_ = false;
        T_newimu_lastimu_prior_.setIdentity();
        have_rotation_prior_ = false;
        have_position_prior_ = false;
        for (auto &frame_vec : overlap_kfs_)
        {
            frame_vec.clear();
        }

        new_frames_.reset();
        last_frames_.reset();
        map_->reset();

        sparse_img_align_->Reset();
        depth_filter_->Reset();
        initializer_->reset();
        
        omg_.setZero();
        vel_.setZero();

        bad_reproj_cnt_ = 0;
        bad_optimize_cnt_ = 0;
        bad_optimize_ = false;
        bad_reproj_ = false;

        VLOG(1) << "SVO RESET ALL";
    }

    void ChannelFrameBase::ResetBackend()
    {
        /*
if (IsBackendValid())
{
#ifdef SVO_LOOP_CLOSING
if (lc_)
{
std::vector<FramePtr> active_kfs;
bundle_adjustment_->getAllActiveKeyframes(&active_kfs);
for (const FramePtr& f: active_kfs)
{
lc_->updateKeyframe(f);
}
}
#endif
bundle_adjustment_->reset();
LOG(WARNING) <<"Resetting backend, the pointer will be reset.";
bundle_adjustment_.reset();
bundle_adjustment_type_ = BundleAdjustmentType::kNone;
}
backend_scale_initialized_ = false;
*/
    }

    int64_t ChannelFrameBase::GetLastFramesTimestamp()
    {
        return (uint64_t)last_frames_->GetMinTimestampNanoseconds();
    }

    std::vector<Transformation> ChannelFrameBase::GetLastFramesCamPose()
    {
        std::vector<Transformation> cams_pose;
        for (size_t i = 0; i < last_frames_->size(); ++i)
        {
            Transformation cam_pose = last_frames_->at(i)->T_world_cam();
            cams_pose.emplace_back(cam_pose);
        }
        return cams_pose;
    }

    Transformation ChannelFrameBase::GetLastFramesIMUPose()
    {
        if(last_frames_ != nullptr && last_frames_->size() > 0)
        {
            Transformation imu_pose = last_frames_->at(0)->T_world_imu();
            return imu_pose;
        }
        else {
            //std::cout<< "reloc last_frames_->size() < 0 or last_frames_ = nullptr" << std::endl;
            LOG_DEBUG_STREAM("reloc last_frames_->size() < 0 or last_frames_ = nullptr");
        }

    }


    void ChannelFrameBase::SetRecovery(const bool recovery)
    {
        loss_without_correction_ = recovery;
#ifdef SVO_LOOP_CLOSING
        if (lc_)
        {
            lc_->setRecoveryMode(recovery);
        }
#endif
#ifdef SVO_GLOBAL_MAP
        if (global_map_)
        {
            global_map_->setEnableReobserved(!recovery);
        }
#endif
    }

    //------------------------------------------------------------------------------
    void ChannelFrameBase::SetTrackingQuality(const size_t num_observations)
    {
        tracking_quality_ = TrackingQuality::kGood;
        if (num_observations < options_.quality_min_fts)
        {
            LOG_WARN_STREAM_THROTTLE(0.5, "Tracking less than "
                                              << options_.quality_min_fts << " features!");
            tracking_quality_ = TrackingQuality::kInsufficient;
        }
        const int feature_drop = static_cast<int>(num_obs_last_) - num_observations;
        // seeds are extracted at keyframe,
        // so the number is not indicative of tracking quality
        if (!last_frames_->IsKeyframe() &&
            feature_drop > options_.quality_max_fts_drop)
        {
            LOG_WARN_STREAM("Lost " << feature_drop << " features!");
            tracking_quality_ = TrackingQuality::kInsufficient;
        }
    }

    //------------------------------------------------------------------------------
    bool ChannelFrameBase::NeedNewKf(const Transformation &)
    {
        //std::cout<< "need new kf -------------------------------------------------------------------aaaaaaaaaaaaaa" << std::endl;
        const std::vector<FramePtr> &visible_kfs = overlap_kfs_.at(0);
        if (options_.kfselect_criterion == KeyframeCriterion::DOWNLOOKING)
        {
            for (const auto &frame : visible_kfs)
            {
                // TODO: does not generalize to multiple cameras!
                Vector3d relpos = new_frames_->at(0)->T_cam_world() * frame->GetCameraPosInWorld();
                if (fabs(relpos.x()) / depth_median_ < options_.kfselect_min_dist && fabs(relpos.y()) / depth_median_ < options_.kfselect_min_dist * 0.8 && fabs(relpos.z()) / depth_median_ < options_.kfselect_min_dist * 1.3)
                    return false;
            }
            //std::cout << "KF Select: NEW KEYFRAME\n";
            return true;
        }

        if (options_.backend_opt && vins_backend_ && vins_backend_->isEstimatorValid())
        {
            if (last_kf_time_sec_ > 0 && options_.kfselect_backend_max_time_sec > 0 &&
                new_frames_->at(0)->GetTimestampSec() - last_kf_time_sec_ >
                    options_.kfselect_backend_max_time_sec)
            {
                return true;
            }
        }
        size_t n_tracked_fts = new_frames_->NumTrackedLandmarks();

        if (n_tracked_fts > options_.kfselect_numkfs_upper_thresh)
        {
            VLOG(40) << "KF Select: NO NEW KEYFRAME Above upper bound";
            return false;
        }

        // TODO: this only works for mono!
        if (last_frames_->at(0)->GetFrameId() - map_->last_added_kf_id_ <
            options_.kfselect_min_num_frames_between_kfs)
        {
            VLOG(40) << "KF Select: NO NEW KEYFRAME We just had a KF";
            return false;
        }

        if (n_tracked_fts < options_.kfselect_numkfs_lower_thresh)
        {
            //std::cout << "KF Select: NEW KEYFRAME Below lower bound\n";
            LOG_DEBUG_STREAM("KF Select: NEW KEYFRAME Below lower bound");
            return true;
        }

        // check that we have at least X disparity w.r.t to last keyframe
        if (options_.kfselect_min_disparity > 0)
        {
            int kf_id = map_->GetLastKeyframe()->GetFrameId();
            std::vector<double> disparities;
            const FramePtr &frame = new_frames_->at(0);
            disparities.reserve(frame->num_features_);
            for (size_t i = 0; i < frame->num_features_; ++i)
            {
                if (frame->landmark_vec_[i])
                {
                    const Point::Point3dObservationVec &observations =
                        frame->landmark_vec_[i]->obs_;
                    for (auto it = observations.rbegin(); it != observations.rend(); ++it)
                    {
                        if (it->frame_id == kf_id)
                        {
                            if (FramePtr kf = it->frame.lock())
                            {
                                disparities.push_back(
                                    (frame->px_vec_.col(i) -
                                     kf->px_vec_.col(it->keypoint_index))
                                        .norm());
                            }
                            break;
                        }
                    }
                }
                // TODO(cfo): loop also over seed references!
            }

            if (!disparities.empty())
            {
                double disparity = vk::getMedian(disparities);
                VLOG(40) << "KF Select: disparity = " << disparity;
                if (disparity < options_.kfselect_min_disparity)
                {
                    VLOG(40) << "KF Select: NO NEW KEYFRAME disparity not large enough";
                    return false;
                }
            }
        }

        for (const auto &kf : visible_kfs)
        {
            // TODO: doesn't generalize to rig!
            const double a =
                Quaternion::Log(new_frames_->at(0)->T_f_w_.GetRotation() *
                                kf->T_f_w_.GetRotation().Inverse())
                    .norm() *
                180 / M_PI;
            const double d = (new_frames_->at(0)->GetCameraPosInWorld() - kf->GetCameraPosInWorld()).norm();
            if (a < options_.kfselect_min_angle && d < options_.kfselect_min_dist_metric)
            {
                VLOG(40) << "KF Select: NO NEW KEYFRAME Min angle = " << a
                         << ", min dist = " << d;
                return false;
            }
        }
        VLOG(40) << "KF Select: NEW KEYFRAME\n";
        return true;
    }
    
    bool ChannelFrameBase::NeedNewKeyframe()
    {
        size_t kf_size = map_->sorted_keyframe_ids_.size();
        if(kf_size < new_frames_->size())
            return false;
        
        size_t total_cnt = 0;
        size_t unconverged_cnt = 0;
        for(size_t cam_idx = 0; cam_idx < new_frames_->size(); ++cam_idx)
        {
            const FramePtr frame = new_frames_->at(cam_idx);
            for(size_t feat_idx = 0; feat_idx < frame->num_features_; ++feat_idx)
            {
                if (frame->landmark_vec_[feat_idx])
                    continue;
                else if (frame->seed_ref_vec_[feat_idx].keyframe)
                {
                    if(isUnconvergedSeed(frame->type_vec_[feat_idx]))
                        ++unconverged_cnt;

                    ++total_cnt;
                }
            }
        }
        
        double ratio = (1.0 * unconverged_cnt) / (total_cnt + 0.1f);
        if(ratio > 0.2) return false;

        std::vector<FramePtr> prv_frames;
        for(size_t i = kf_size - 1; i >= 0; --i)
        {
            const FramePtr &last_keyframe = map_->GetKeyFrameAt(i);
            prv_frames.emplace_back(last_keyframe);
            if(prv_frames.size() >= new_frames_->size())
                break;
        }

        double total_judgement = 0.0f;
        for(const FramePtr &prv_frame : prv_frames)
        {
            int img_width = prv_frame->cam_->imageWidth();
            int img_height = prv_frame->cam_->imageHeight();

            int prv_cam_idx = prv_frame->nframe_index_;
            Transformation prv_T_w_c = prv_frame->T_world_cam();
            
            for(size_t cam_idx = 0; cam_idx < new_frames_->size(); ++cam_idx)
            {
                const FramePtr &cur_frame = new_frames_->at(cam_idx);
                int cur_cam_idx = cur_frame->nframe_index_;

                if(prv_cam_idx != cur_cam_idx)
                  continue;
                
                Eigen::Quaterniond QI = Eigen::Quaterniond::Identity();
                Transformation cur_T_w_c = cur_frame->T_world_cam();
                Transformation T_cur_prv = cur_T_w_c.Inverse() * prv_T_w_c;
                Transformation T_cur_prv2(-T_cur_prv.GetPosition(), T_cur_prv.GetRotation());
                Transformation T_cur_prv_ir(T_cur_prv.GetPosition(), QI);
                Transformation T_cur_prv_ir2(-T_cur_prv.GetPosition(), QI);

                float optical_flow = 0.0f, optical_flow_ir = 0.0f;
                size_t total_cnt = 0; 

                for(size_t feat_idx = 0; feat_idx < prv_frame->NumFeatures(); ++feat_idx)
                {
                    if(!prv_frame->landmark_vec_[feat_idx])
                        continue;
                    
                    Eigen::Vector2d px_ref = prv_frame->px_vec_.col(feat_idx);
                    const PointPtr &pt = prv_frame->landmark_vec_[feat_idx];
                    Eigen::Vector3d pt_ref = prv_frame->f_vec_.col(feat_idx) 
                                      * (pt->pos() - prv_frame->GetCameraPosInWorld()).norm();
                    Eigen::Vector3d pt_cur = T_cur_prv * pt_ref;
                    Eigen::Vector3d pt_cur2 = T_cur_prv2 * pt_ref;
                    Eigen::Vector3d pt_cur_ir = T_cur_prv_ir * pt_ref;
                    Eigen::Vector3d pt_cur_ir2 = T_cur_prv_ir2 * pt_ref;
                    
                    Eigen::Vector3d f_cur = pt_cur.normalized();
                    Eigen::Vector3d f_cur2 = pt_cur2.normalized();
                    Eigen::Vector3d f_cur_ir = pt_cur_ir.normalized();
                    Eigen::Vector3d f_cur_ir2 = pt_cur_ir2.normalized();

                    Eigen::Vector2d px_cur, px_cur_ir;
                    Eigen::Vector2d px_cur2, px_cur_ir2;

                    cur_frame->cam_->project3(f_cur, &px_cur);
                    cur_frame->cam_->project3(f_cur2, &px_cur2);
                    cur_frame->cam_->project3(f_cur_ir, &px_cur_ir);
                    cur_frame->cam_->project3(f_cur_ir2, &px_cur_ir2);

                    optical_flow += (px_cur - px_ref).squaredNorm();
                    optical_flow += (px_cur2 - px_ref).squaredNorm();

                    optical_flow_ir += (px_cur_ir - px_ref).squaredNorm();
                    optical_flow_ir += (px_cur_ir2 - px_ref).squaredNorm();

                    total_cnt += 2;
                }

                optical_flow /= (total_cnt + 0.1f);
                optical_flow = sqrtf(optical_flow);

                optical_flow_ir /= (total_cnt + 0.1f); 
                optical_flow_ir = sqrtf(optical_flow_ir);

                const float global_weight = 1.0;
                const float max_shift_weight_T = 0.04;
                const float max_shift_weight_RT = 0.02;

                float judgement = global_weight * max_shift_weight_T * optical_flow_ir
                            + global_weight * max_shift_weight_RT * optical_flow;

                total_judgement += judgement;
            }
        }

        total_judgement /= (prv_frames.size() + 0.1f);

        // ROS_WARN("needNewKeyFrame: unconverged ratio: %.4lf", ratio);
        // ROS_WARN("needNewKeyFrame: total judgement: %.4lf", total_judgement);

        return total_judgement > 1.0f;
    }

    void ChannelFrameBase::GetMotionPrior(const bool /*use_velocity_in_frame*/)
    {
        if (have_rotation_prior_ && !options_.use_imu_only_for_gravityalign)
        {
            VLOG(40) << "Get motion prior from provided rotation prior.";
            LOG_DEBUG_STREAM("imu:rotation prior");
            if(last_frames_ && new_frames_)
            {
                double delta_ts = new_frames_->GetMinTimestampSeconds()
                              - last_frames_->GetMinTimestampSeconds();
                t_lastimu_newimu_ = vel_ * delta_ts;
            }
            T_newimu_lastimu_prior_ = Transformation(R_imulast_world_ * R_imu_world_.Inverse(), t_lastimu_newimu_).Inverse();
            have_motion_prior_ = true;
        }
        else if(last_frames_ && new_frames_
          && vel_ != Eigen::Vector3d::Zero() 
          && omg_ != Eigen::Vector3d::Zero())
        {
          VLOG(40) << "Get motion prior with constant speed model.";
          double delta_ts = new_frames_->GetMinTimestampSeconds()
                        - last_frames_->GetMinTimestampSeconds();
          t_lastimu_newimu_ = vel_ * delta_ts;
          Eigen::Vector3d delta_theta = omg_ * delta_ts; 
          Eigen::Quaterniond q_lastimu_newimu_(1.0f, 0.5f * delta_theta.x(), 
                              0.5f * delta_theta.y(), 0.5f * delta_theta.z());

          q_lastimu_newimu_.normalize();
          T_newimu_lastimu_prior_ = Transformation(q_lastimu_newimu_, t_lastimu_newimu_).Inverse();
          have_motion_prior_ = true;
        }
        else if (new_frames_->imu_timestamps_ns_.cols() > 0 && !options_.use_imu_only_for_gravityalign)
        {
            VLOG(40) << "Get motion prior from integrated IMU measurements.";
            const Eigen::Matrix<int64_t, 1, Eigen::Dynamic> &imu_timestamps_ns =
                new_frames_->imu_timestamps_ns_;
            const Eigen::Matrix<double, 6, Eigen::Dynamic> &imu_measurements =
                new_frames_->imu_measurements_;
            Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero(); // TODO!
            const size_t num_measurements = imu_timestamps_ns.cols();
            Quaternion delta_R;
            for (size_t m_idx = 0u; m_idx < num_measurements - 1u; ++m_idx)
            {
                const double delta_t_seconds =
                    (imu_timestamps_ns(m_idx + 1) - imu_timestamps_ns(m_idx)) * common::conversions::kNanoSecondsToSeconds;
                CHECK_LE(delta_t_seconds, 1e-12) << "IMU timestamps need to be strictly increasing.";

                const Eigen::Vector3d w = imu_measurements.col(m_idx).tail<3>() - gyro_bias;
                const Quaternion R_incr = Quaternion::Exp(w * delta_t_seconds);
                delta_R = delta_R * R_incr;
            }
            T_newimu_lastimu_prior_ = Transformation(delta_R, t_lastimu_newimu_).Inverse();
            have_motion_prior_ = true;
        }
        /*
else if(imu_handler_)
{
ImuMeasurements imu_measurements;
if(!imu_handler_->GetMeasurements(
last_frames_->GetTimestampSec(), new_frames_->GetTimestampSec(), false, imu_measurements))
return false;

PreintegratedImuMeasurement preint(
//      last_frames_->omega_bias_, last_frames_->omega_bias_); TODO: check why this worked before
imuHandler()->GetGyroscopeBias(), imuHandler()->GetAccelerometerBias());
preint.AddMeasurements(imu_measurements);

Vector3d t_last_new = t_lastimu_newimu_;
if(use_velocity_in_frame)
{
t_last_new =
preint.delta_t_ij_
+ last_frames_->T_f_w_.GetRotation().Rotate(
last_frames_->velocity_*preint.dt_sum_
- Vector3d(0, 0, -imu_handler_->imu_calib_.gravity_magnitude)*0.5*preint.dt_sum_*preint.dt_sum_);
}

Quaternion R_lastimu_newimu;
imu_handler_->GetRelativeRotationPrior(
last_frames_->getTimestampSec(), new_frames_->getTimestampSec(), false, R_lastimu_newimu);
T_newimu_lastimu_prior_ = Transformation(R_lastimu_newimu, t_last_new).Inverse();
return true;
}
*/
        else if (options_.poseoptim_prior_lambda > 0 || options_.img_align_prior_lambda_rot > 0 || options_.img_align_prior_lambda_trans > 0)
        {
            VLOG(40) << "Get motion prior by assuming constant velocity.";
            T_newimu_lastimu_prior_ = Transformation(Quaternion(), t_lastimu_newimu_).Inverse();
            have_motion_prior_ = true;
        }
        return;
    }

    //------------------------------------------------------------------------------
    void ChannelFrameBase::SetDetectorOccupiedCells(
        const size_t reprojector_grid_idx, const DetectorPtr &feature_detector)
    {
        const Reprojector &rep = *reprojectors_.at(reprojector_grid_idx);
        CHECK_EQ(feature_detector->grid_.size(), rep.grid_->size());
        feature_detector->grid_.occupancy_ = rep.grid_->occupancy_;
        if (rep.fixed_landmark_grid_)
        {
            for (size_t idx = 0; idx < rep.fixed_landmark_grid_->size(); idx++)
            {
                if (rep.fixed_landmark_grid_->isOccupied(idx))
                {
                    feature_detector->grid_.occupancy_[idx] = true;
                }
            }
        }
    }

    void ChannelFrameBase::SetFirstFrames(const std::vector<FramePtr> &first_frames)
    {
        ResetAll();
        last_frames_.reset(new FrameBundle(first_frames));
        for (auto f : last_frames_->frames_)
        {
            f->SetKeyframe();
            map_->AddKeyframe(f, bundle_adjustment_type_ == BundleAdjustmentType::kCeres);
        }
        stage_ = Stage::kTracking;
    }

    std::vector<FramePtr> ChannelFrameBase::CloseKeyframes() const
    {
        std::vector<FramePtr> close_kfs;
        for (const auto &kfs : overlap_kfs_)
        {
            close_kfs.insert(close_kfs.begin(), kfs.begin(), kfs.end());
        }
        return close_kfs;
    }

    void ChannelFrameBase::SetCompensation(const bool do_compensation)
    {
        sparse_img_align_->SetCompensation(do_compensation);
        for (const ReprojectorPtr &rep : reprojectors_)
        {
            rep->options_.affine_est_gain = do_compensation;
        }
        depth_filter_->GetMatcher().m_patch_matcher_options.affine_est_gain_ = do_compensation;
    }

    void ChannelFrameBase::SetVinsBackend(const BundleAdjustmentPtr &ba)
    {
        vins_backend_ = ba;
        window_size_ = vins_backend_->getWindowsSize();
    }

    bool ChannelFrameBase::IsLastKeyFrame()
    {
      if(!last_frames_)
          return false;
        
      for(size_t i = 0; i < last_frames_->size(); ++i)
          if(last_frames_->at(0)->is_keyframe_)
              return true;
      return false;
    }
    
    double ChannelFrameBase::GetLastFramesTimestampSec()
    {
        return last_frames_->GetMinTimestampSeconds();
    }
    
    Transformation ChannelFrameBase::GetLastFramesImuPose()
    {
        Transformation imu_pose = last_frames_->Get_T_W_B();
        return imu_pose;
    }
    
    int ChannelFrameBase::getAbnormalCount()
    {
        return abnormal_cnt_;
    }
    
    void ChannelFrameBase::SetAbnormalResult(bool is_abnormal) 
    {
        is_abnormal_ = is_abnormal; 
    }

    void ChannelFrameBase::SetAbnormalPose(const Transformation& abnormal_pose) 
    { 
        abnormal_pose_ = abnormal_pose; 
    }
    
    // Set motion prior for new frames
    void ChannelFrameBase::SetMotionPrior(const FrameBundlePtr& frame_bundle)
    {   
        // if(stage_ == Stage::kInitializing)
        //     return;

        if(vins_backend_)
            vins_backend_->getMotionPrior(new_frames_, last_frames_, have_motion_prior_);
        if (have_motion_prior_)
        {
            have_rotation_prior_ = true;
            R_imu_world_ = new_frames_->Get_T_W_B().Inverse().GetRotation();
            if (last_frames_)
            {
                T_newimu_lastimu_prior_ = new_frames_->Get_T_W_B().Inverse() * last_frames_->Get_T_W_B();
                have_motion_prior_ = true;
            }
        }
        else
        {
            // Predict pose of new frame using motion prior.
            // TODO(cfo): remove same from ProcessFrame in mono.
            if (last_frames_)
            {
                VLOG(40) << "Predict pose of new image using motion prior.";
                GetMotionPrior(false); //use imu predict position

                // set initial pose estimate
                for (size_t i = 0; i < new_frames_->size(); ++i)
                {
                    new_frames_->at(i)->T_f_w_ = new_frames_->at(i)->T_cam_imu() * T_newimu_lastimu_prior_ * last_frames_->at(i)->T_imu_world();
                }
            }
        }
    }
    
    // Calc vel and omg for constant speed model
    void ChannelFrameBase::calcVelAndOmg()
    {
        if(!last_frames_ || !new_frames_)
           return;

        Transformation delta_T = last_frames_->Get_T_W_B().Inverse()
                                  * new_frames_->Get_T_W_B();

        double delta_ts = new_frames_->GetMinTimestampSeconds()
                          - last_frames_->GetMinTimestampSeconds();

        Eigen::Matrix3d delta_R = delta_T.GetRotationMatrix();
        Eigen::Vector3d delta_t = delta_T.GetPosition();
        Eigen::Quaterniond delta_q(delta_R);
        Eigen::Vector3d delta_theta = 2.0f * delta_q.vec();

        vel_ = delta_t / delta_ts;
        omg_ = delta_theta / delta_ts;

        // float alpha = 0.5f;
        // omg_ = (1 - alpha) * omg_ + (alpha / delta_ts) * delta_theta;
        // vel_ = (1 - alpha) * vel_ + (alpha / delta_ts) * delta_t;

        float max_vel = 1.5f;
        if(vel_.norm() > max_vel)
           vel_.setZero();
    }
    
    bool ChannelFrameBase::isFrontendOk()
    {
      return !frontend_abnormal_;
    }
    
    bool ChannelFrameBase::isBackendOK()
    {
        if(!vins_backend_)
            return true;

        return vins_backend_->isBackendOK();
    }

    void ChannelFrameBase::ResetAllEnd()
    {
      ++abnormal_cnt_;
      ResetVisionFrontendCommon();

      if(vins_backend_)
        vins_backend_->reset();
    }
    
} // namespace mivins
