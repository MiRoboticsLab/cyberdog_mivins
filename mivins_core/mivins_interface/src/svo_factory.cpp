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

#include <mivins/mivins_global_headers.h>
#include <svo/param.h>
#include <mivins/common/imu_calibration.h>
#include <svo/svo_factory.h>
#include <svo/loose_couple.h>
#include <svo/pose_update.h>
//#include <mivins/frame_handler_rgbd.h>
#include <mivins/channel_frame_mono.h>
#include <mivins/channel_frame_stereo.h>
//#include <mivins/frame_handler_triple_with_stereo.h>
#include <mivins/channel_frame_rgbdfisheye.h>

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <yaml-cpp/yaml.h>
#pragma diagnostic pop

namespace mivins
{
    namespace factory
    {

        BaseOptions loadBaseOptions(const std::string config_file, bool forward_default)
        {
            BaseOptions o;
            YAML::Node config = YAML::LoadFile(config_file);
            o.max_n_kfs = getParam<int>(config, "max_n_kfs", 5);
            o.use_photometric_calibration = getParam<bool>(config, "use_photometric_calibration", false);
            o.use_joint_init = getParam<bool>(config, "use_joint_init", true);
            o.backend_opt = getParam<bool>(config, "backend_opt", false);
            //o.use_backend_init = getParam<bool>(config, "use_backend_init", false);
            o.use_imu = getParam<bool>(config, "use_imu", false);
            o.use_imu_only_for_gravityalign = getParam<bool>(config, "use_imu_only_for_gravityalign", false);
            o.trace_dir = getParam<std::string>(config, "trace_dir", ""); // WCP TODO
            o.init_min_fts = getParam<int>(config, "init_min_fts", 100);
            o.quality_min_fts = getParam<int>(config, "quality_min_fts", 50);
            o.quality_max_fts_drop = getParam<int>(config, "quality_max_drop_fts", 40);
            o.relocalization_max_trials = getParam<int>(config, "relocalization_max_trials", 50);
            o.poseoptim_prior_lambda = getParam<double>(config, "poseoptim_prior_lambda", 0.0);
            o.poseoptim_using_unit_sphere = getParam<bool>(config, "poseoptim_using_unit_sphere", false);
            o.img_align_prior_lambda_rot = getParam<double>(config, "img_align_prior_lambda_rot", 0.0);
            o.img_align_prior_lambda_trans = getParam<double>(config, "img_align_prior_lambda_trans", 0.0);
            o.structure_optimization_max_pts = getParam<int>(config, "structure_optimization_max_pts", 20);
            o.init_map_scale = getParam<double>(config, "map_scale", 1.0);
            std::string default_kf_criterion = forward_default ? "FORWARD" : "DOWNLOOKING";
            if (getParam<std::string>(config, "kfselect_criterion", default_kf_criterion) == "FORWARD")
                o.kfselect_criterion = KeyframeCriterion::FORWARD;
            else
                o.kfselect_criterion = KeyframeCriterion::DOWNLOOKING;
            o.kfselect_min_dist = getParam<double>(config, "kfselect_min_dist", 0.12);
            o.kfselect_numkfs_upper_thresh = getParam<int>(config, "kfselect_numkfs_upper_thresh", 120);
            o.kfselect_numkfs_lower_thresh = getParam<double>(config, "kfselect_numkfs_lower_thresh", 70);
            o.kfselect_min_dist_metric = getParam<double>(config, "kfselect_min_dist_metric", 0.01);
            o.kfselect_min_angle = getParam<double>(config, "kfselect_min_angle", 20);
            o.kfselect_min_disparity = getParam<double>(config, "kfselect_min_disparity", 40);
            o.kfselect_min_num_frames_between_kfs = getParam<int>(config, "kfselect_min_num_frames_between_kfs", 2);
            o.kfselect_backend_max_time_sec = getParam<double>(config, "kfselect_backend_max_time_sec", 3.0);
            o.img_align_max_level = getParam<int>(config, "img_align_max_level", 4);
            o.img_align_min_level = getParam<int>(config, "img_align_min_level", 2);
            o.img_align_robustification = getParam<bool>(config, "img_align_robustification", false);
            o.img_align_use_distortion_jacobian =
                getParam<bool>(config, "img_align_use_distortion_jacobian", false);
            // o.img_align_est_ab = getParam<bool>(config, "img_align_est_ab", false);
            // o.img_align_est_illumination_gain =
            //     getParam<bool>(config, "img_align_est_illumination_gain", false);
            // o.img_align_est_illumination_offset =
            //     getParam<bool>(config, "img_align_est_illumination_offset", false);
            o.img_align_estimate_ab = getParam<bool>(config, "img_align_est_ab", false);
            o.img_align_estimate_alpha =
                getParam<bool>(config, "img_align_est_illumination_gain", false);
            o.img_align_estimate_beta =
                getParam<bool>(config, "img_align_est_illumination_offset", false);

            o.poseoptim_thresh = getParam<double>(config, "poseoptim_thresh", 2.0);
            o.update_seeds_with_old_keyframes =
                getParam<bool>(config, "update_seeds_with_old_keyframes", true);
            o.use_async_reprojectors = getParam<bool>(config, "use_async_reprojectors", false);
            o.trace_statistics = getParam<bool>(config, "trace_statistics", false);
            o.trace_dir = getParam<std::string>(config, "trace_dir", "/tmp/");
            o.save_pose_path = getParam<std::string>(config, "save_pose_path", "/SDCARD/miloc/maps/default/visual/trajectory.txt");
            o.save_kf_pose_path = getParam<std::string>(config, "save_kf_pose_path", "/SDCARD/miloc/maps/default/visual/kf_trajectory.txt");
            o.light_optimize = getParam<bool>(config, "light_optimize", true);
            // o.backend_scale_stable_thresh =
            //     getParam<double>(config, "backend_scale_stable_thresh", 0.02);
            // o.global_map_lc_timeout_sec_ =
            //     getParam<double>(config, "global_map_timeout_sec", 2.0);
            return o;
        }

        DetectorOptions loadDetectorOptions(const std::string config_file)
        {
            DetectorOptions o;
            YAML::Node config = YAML::LoadFile(config_file);
            o.cell_size = getParam<int>(config, "grid_size", 35);
            o.max_level = getParam<int>(config, "n_pyr_levels", 3) - 1;
            o.threshold_primary = getParam<int>(config, "detector_threshold_primary", 10);
            o.threshold_secondary = getParam<int>(config, "detector_threshold_secondary", 200);
            o.threshold_shitomasi = getParam<int>(config, "detector_threshold_shitomasi", 100);

            std::string detector_type = getParam<std::string>(config, "detector_type", "Fast");
            if(detector_type == "Fast")
            {
                if(getParam<bool>(config, "use_edgelets", true))
                    o.detector_type = DetectorType::kFastGrad;
                else
                    o.detector_type = DetectorType::kFast;
            }
            else if(detector_type == "ShiTomasi")
            {
                if(getParam<bool>(config, "use_edgelets", true))
                    o.detector_type = DetectorType::kShiTomasiGrad;
                else
                    o.detector_type = DetectorType::kShiTomasi;
            }
            else if(detector_type == "Grad")
            {
                if(getParam<bool>(config, "use_edgelets", true))
                    o.detector_type = DetectorType::kGridGrad;
                else
                    o.detector_type = DetectorType::kGrad;
            }
            else if(detector_type == "All")
            {
                o.detector_type = DetectorType::kAll;
            }
            else if(detector_type == "GradHuangMumford")
            {
                o.detector_type = DetectorType::kGradHuangMumford;
            }
            else if(detector_type == "Canny")
            {
                o.detector_type = DetectorType::kCanny;
            }
            else if(detector_type == "Sobel")
            {
                if(getParam<bool>(config, "use_edgelets", true))
                   o.detector_type = DetectorType::kSobelGrad;
                else
                   o.detector_type = DetectorType::kSobel;
            }
            return o;
        }

        DepthOptimizationOptions loadDepthOptimizationOptions(const std::string config_file)
        {
            DepthOptimizationOptions o;
            YAML::Node config = YAML::LoadFile(config_file);
            o.max_search_level = getParam<int>(config, "n_pyr_levels", 3) - 1;
            o.use_threaded_depthfilter =
                getParam<bool>(config, "use_threaded_depthfilter", true);
            o.seed_convergence_sigma2_thresh =
                getParam<double>(config, "seed_convergence_sigma2_thresh", 200.0);
            o.mappoint_convergence_sigma2_thresh =
                getParam<double>(config, "mappoint_convergence_sigma2_thresh", 500.0);
            o.scan_epi_unit_sphere = getParam<bool>(config, "scan_epi_unit_sphere", false);
            o.affine_est_offset = getParam<bool>(config, "depth_filter_affine_est_offset", true);
            o.affine_est_gain = getParam<bool>(config, "depth_filter_affine_est_gain", false);
            o.use_init_depth = getParam<bool>(config, "depth_filter_use_init_depth", false);

            o.max_n_seeds_per_frame = static_cast<size_t>(
                static_cast<double>(getParam<int>(config, "max_fts", 120)) * getParam<double>(config, "max_seeds_ratio", 3.0));
            o.max_map_seeds_per_frame = static_cast<size_t>(
                static_cast<double>(getParam<int>(config, "max_map_fts", 120)));
            o.extra_map_points =
                getParam<bool>(config, "depth_filter_extra_map_points", false);
            if (getParam<bool>(config, "runlc", false) && !o.extra_map_points)
            {
                LOG(WARNING) << "Loop closure requires extra map points, "
                             << " but the option is not set, overriding to true.";
                o.extra_map_points = true;
            }
            o.trace_statistics = getParam<bool>(config, "trace_statistics", false);
            o.trace_dir = getParam<std::string>(config, "trace_dir", "/tmp/");
            return o;
        }

        InitializationOptions loadInitializationOptions(const std::string config_file)
        {
            InitializationOptions o;
            YAML::Node config = YAML::LoadFile(config_file);
            o.init_min_features = getParam<int>(config, "init_min_features", 100);
            o.init_min_tracked = getParam<int>(config, "init_min_tracked", 80);
            o.init_min_inliers = getParam<int>(config, "init_min_inliers", 70);
            o.init_min_disparity = getParam<double>(config, "init_min_disparity", 40.0);
            o.init_min_features_factor = getParam<double>(config, "init_min_features_factor", 2.0);
            o.reproj_error_thresh = getParam<double>(config, "reproj_err_thresh", 2.0);
            o.init_disparity_pivot_ratio = getParam<double>(config, "init_disparity_pivot_ratio", 0.5);
            std::string init_method = getParam<std::string>(config, "init_method", "FivePoint");
            if (init_method == "Homography")
                o.init_type = InitializerType::kHomography;
            else if (init_method == "TwoPoint")
                o.init_type = InitializerType::kTwoPoint;
            else if (init_method == "FivePoint")
                o.init_type = InitializerType::kFivePoint;
            else if (init_method == "OneShot")
                o.init_type = InitializerType::kOneShot;
            else
                SVO_ERROR_STREAM("Initialization Method not supported: " << init_method);
            return o;
        }

        FeatureTrackerOptions loadTrackerOptions(const std::string config_file)
        {
            FeatureTrackerOptions o;
            YAML::Node config = YAML::LoadFile(config_file);
            o.klt_max_level = getParam<int>(config, "klt_max_level", 4);
            o.klt_min_level = getParam<int>(config, "klt_min_level", 0.001);
            o.init_features_size = getParam<int>(config, "init_features_size", 150);
            return o;
        }

        ReprojectorOptions loadReprojectorOptions(const std::string config_file)
        {
            ReprojectorOptions o;
            YAML::Node config = YAML::LoadFile(config_file);
            o.max_n_kfs = getParam<int>(config, "reprojector_max_n_kfs", 5);
            o.max_n_features_per_frame = getParam<int>(config, "max_fts", 160);
            o.cell_size = getParam<int>(config, "grid_size", 35);
            o.reproject_unconverged_seeds =
                getParam<bool>(config, "reproject_unconverged_seeds", true);
            o.max_unconverged_seeds_ratio =
                getParam<double>(config, "max_unconverged_seeds_ratio", -1.0);
            o.min_required_features =
                getParam<int>(config, "quality_min_fts", 50);
            o.seed_sigma2_thresh =
                getParam<double>(config, "seed_convergence_sigma2_thresh", 200.0);

            o.affine_est_offset =
                getParam<bool>(config, "reprojector_affine_est_offset", true);
            o.affine_est_gain =
                getParam<bool>(config, "reprojector_affine_est_gain", false);
            o.max_fixed_landmarks =
                getParam<int>(config, "reprojector_max_fixed_landmarks", 50);
            o.max_n_global_kfs =
                getParam<int>(config, "reprojector_max_n_global_kfs", 20);
            o.use_kfs_from_global_map =
                getParam<bool>(config, "reprojector_use_kfs_from_global_map", false);
            o.fixed_lm_grid_size =
                getParam<int>(config, "reprojector_fixed_lm_grid_size", 50);

            return o;
        }
        
        LooseCoupleOptions loadLooseCoupleOptions(const std::string config_file)
        {
            LooseCoupleOptions o;
            YAML::Node config = YAML::LoadFile(config_file);
            
            o.kf_vio_size = getParam<int>(config, "kf_vio_size", 50);
            o.init_time_thresh = getParam<float>(config, "init_time_thresh", 3.0f);

            o.ab_delta_yaw = getParam<float>(config, "ab_delta_yaw", 30.0f);
            o.ab_dist_ratio = getParam<float>(config, "ab_dist_ratio", 1.5f);
            o.ab_backtrack_ts = getParam<float>(config, "ab_backtrack_ts", 3.0f);
            o.ab_continuous_cnt = getParam<int>(config, "ab_continuous_cnt", 3);

            o.use_imu_rot = getParam<bool>(config, "use_imu_rot", false);

            return o;
        }

        PoseUpdateOptions loadPoseUpdateOptions(const std::string config_file)
        {
            PoseUpdateOptions o;
            YAML::Node config = YAML::LoadFile(config_file);

            o.ini_pose_num = getParam<int>(config, "ini_pose_num", 1000);
            o.adj_pose_num = getParam<int>(config, "adj_pose_num", 500);
            o.opt_pose_num = getParam<int>(config, "opt_pose_num", 100);
            o.compensate_z = getParam<bool>(config, "compensate_z", false);
            o.adjust_pose = getParam<bool>(config, "adjust_pose", false);
            o.adj_z_n = getParam<int>(config, "adj_z_n", 30);

            return o;
        }

        CameraBundle::Ptr loadCameraFromYaml(const std::string calib_file)
        {
            // std::string calib_file = getParam<std::string>(pnh, "calib_file", "~/cam.yaml");
            CameraBundle::Ptr ncam = CameraBundle::loadFromYaml(calib_file);
            std::cout << "loaded " << ncam->numCameras() << " cameras";
            for (const auto &cam : ncam->getCameraVector())
                cam->printParameters(std::cout, "");//
            return ncam;
        }

        // #ifdef SVO_LOOP_CLOSING
        // MapAlignmentOptions loadMapAlignmentOptions(const std::string config_file)
        // {
        //   MapAlignmentOptions o;
        //   YAML::Node config = YAML::LoadFile(config_file);
        //   o.ransac3d_inlier_percent =
        //       getParam<double>(config, "ransac3d_inlier_percent", 40.0);
        //   o.ransac3d_min_pts = getParam<int>(config, "ransac3d_min_pts", 8);

        //   return o;
        // }

        StereoTriangulationOptions loadStereoOptions(const std::string config_file)
        {
            StereoTriangulationOptions o;
            YAML::Node config = YAML::LoadFile(config_file);
            o.triangulate_n_features = getParam<int>(config, "max_fts", 120);
            o.max_depth_inv = getParam<double>(config, "max_depth_inv", 1.0 / 50.0);
            o.min_depth_inv = getParam<double>(config, "min_depth_inv", 1.0 / 0.5);
            o.mean_depth_inv = getParam<double>(config, "mean_depth_inv", 1.0 / 2.0);
            return o;
        }

        ChannelImu::Ptr getImuHandler(const std::string config_file,
                                      const std::string calib_file)
        {
            // std::string calib_file = getParam<std::string>(pnh, "calib_file", "");
            ImuCalibration imu_calib = ChannelImu::LoadCalibrationFromFile(calib_file);
            imu_calib.print("Loaded IMU Calibration");
            ImuInitialization imu_init = ChannelImu::LoadInitializationFromFile(calib_file);
            imu_init.print("Loaded IMU Initialization");
            IMUHandlerOptions options;
            YAML::Node config = YAML::LoadFile(config_file);
            options.temporal_stationary_check =
                getParam<bool>(config, "imu_temporal_stationary_check", false);
            options.temporal_window_length_sec_ =
                getParam<double>(config, "imu_temporal_window_length_sec", 0.5);
            options.stationary_acc_sigma_thresh_ =
                getParam<double>(config, "stationary_acc_sigma_thresh", 0.0);
            options.stationary_gyr_sigma_thresh_ =
                getParam<double>(config, "stationary_gyr_sigma_thresh", 0.0);
            ChannelImu::Ptr imu_handler(new ChannelImu(imu_calib, imu_init, options));
            return imu_handler;
        }

        ChannelOdom::Ptr getOdomHandler(const std::string calib_file)
        {
            // std::string calib_file = getParam<std::string>(pnh, "calib_file", "");
            OdomCalibration odom_calib = ChannelOdom::LoadCalibrationFromFile(calib_file);
            odom_calib.print("Loaded ODOM Calibration");

            ChannelOdom::Ptr odom_handler(new ChannelOdom(odom_calib));
            return odom_handler;
        }

        LooseCouple::Ptr getLooseCoupleHandler(const std::string config_file, 
                                               const std::string calib_file)
        {
            OdomCalibration odom_calib = ChannelOdom::LoadCalibrationFromFile(calib_file);
            Eigen::Matrix4d T_b_o = odom_calib.T_B_O.GetTransformationMatrix();
            
            LooseCoupleOptions options = loadLooseCoupleOptions(config_file);
            LooseCouple::Ptr loose_couple_ptr_(new LooseCouple(options, T_b_o));
            return loose_couple_ptr_;
        }

        PoseUpdate::Ptr getPoseUpdateHandler(const std::string config_file)
        {
            PoseUpdateOptions options = loadPoseUpdateOptions(config_file);
            PoseUpdate::Ptr pose_update_ptr_(new PoseUpdate(options));

            return pose_update_ptr_;
        }

        void SetInitialPose(const std::string config_file, ChannelFrameBase& vo)
        {
            YAML::Node config = YAML::LoadFile(config_file);

            Transformation T_world_imuinit(
                Quaternion(getParam<double>(config, "T_world_imuinit/qw", 1.0),
                           getParam<double>(config, "T_world_imuinit/qx", 0.0),
                           getParam<double>(config, "T_world_imuinit/qy", 0.0),
                           getParam<double>(config, "T_world_imuinit/qz", 0.0)),
                Vector3d(getParam<double>(config, "T_world_imuinit/tx", 0.0),
                         getParam<double>(config, "T_world_imuinit/ty", 0.0),
                         getParam<double>(config, "T_world_imuinit/tz", 0.0)));
            vo.SetInitialImuPose(T_world_imuinit);
        }

        // FrameHandlerRgbd::Ptr makeRgbd(const std::string config_file,
        //   const std::string calib_file, const CameraBundlePtr& cam)
        // {
        //   // Create camera
        //   CameraBundle::Ptr ncam = (cam) ? cam : loadCameraFromYaml(calib_file);
        //   if (ncam->numCameras() > 1)
        //   {
        //     LOG(WARNING) << "Load more cameras than needed, will erase from the end.";
        //     ncam->keepFirstNCams(1);
        //   }

        //   // Init VO
        //   FrameHandlerRgbd::Ptr vo =
        //       std::make_shared<FrameHandlerRgbd>(
        //         loadBaseOptions(config_file, true),
        //         loadDepthOptimizationOptions(config_file),
        //         loadDetectorOptions(config_file),
        //         loadInitializationOptions(config_file),
        //         loadReprojectorOptions(config_file),
        //         loadTrackerOptions(config_file),
        //         ncam);

        //   // Get initial position and orientation of IMU
        //   SetInitialPose(config_file, *vo);

        //   return vo;
        // }

        ChannelFrameMono::Ptr makeMono(const std::string config_file,
                                       const std::string calib_file, const CameraBundlePtr& cam)
        {
            // Create camera
            CameraBundle::Ptr ncam = (cam) ? cam : loadCameraFromYaml(calib_file);
            if (ncam->numCameras() > 1)
            {
                LOG(WARNING) << "Load more cameras than needed, will erase from the end.";
                ncam->keepFirstNCams(1);
            }

            // Init VO
            ChannelFrameMono::Ptr vo =
                std::make_shared<ChannelFrameMono>(
                    loadBaseOptions(config_file, true),
                    loadDepthOptimizationOptions(config_file),
                    loadDetectorOptions(config_file),
                    loadInitializationOptions(config_file),
                    loadReprojectorOptions(config_file),
                    loadTrackerOptions(config_file),
                    ncam);

            // Get initial position and orientation of IMU
            SetInitialPose(config_file, *vo);

            return vo;
        }

        ChannelFrameStereo::Ptr makeStereo(const std::string config_file,
                                           const std::string calib_file, const CameraBundlePtr& cam)
        {
            // Load cameras
            CameraBundle::Ptr ncam = (cam) ? cam : loadCameraFromYaml(calib_file);
            if (ncam->numCameras() > 2)
            {
                LOG(WARNING) << "Load more cameras than needed, will erase from the end.";
                ncam->keepFirstNCams(2);
            }

            // Init VO
            InitializationOptions init_options = loadInitializationOptions(config_file);
            init_options.init_type = InitializerType::kStereo;
            ChannelFrameStereo::Ptr vo =
                std::make_shared<ChannelFrameStereo>(
                    loadBaseOptions(config_file, true),
                    loadDepthOptimizationOptions(config_file),
                    loadDetectorOptions(config_file),
                    init_options,
                    loadStereoOptions(config_file),
                    loadReprojectorOptions(config_file),
                    loadTrackerOptions(config_file),
                    ncam);

            // Get initial position and orientation of IMU
            SetInitialPose(config_file, *vo);

            return vo;
        }

        // FrameHandlerTripleWithStereo::Ptr makeTripleWithStereo(const std::string config_file,
        //                             const std::string calib_file, const CameraBundlePtr& cam)
        // {
        //   CameraBundle::Ptr ncam = (cam) ? cam : loadCameraFromYaml(calib_file);
        //   if (ncam->numCameras() > 3)
        //   {
        //     LOG(WARNING) << "Load more cameras than needed, will erase from the end.";
        //     ncam->keepFirstNCams(3);
        //   }

        //   InitializationOptions init_options = loadInitializationOptions(config_file);
        //   // cameras: one stereo camera, and one mono camera
        //   init_options.init_type = InitializerType::kStereo;
        //   FrameHandlerTripleWithStereo::Ptr vo =
        //       std::make_shared<FrameHandlerTripleWithStereo>(
        //         loadBaseOptions(config_file, true),
        //         loadDepthOptimizationOptions(config_file),
        //         loadDetectorOptions(config_file),
        //         init_options,
        //         loadStereoOptions(config_file),
        //         loadReprojectorOptions(config_file),
        //         loadTrackerOptions(config_file),
        //         ncam);

        //   // Get initial position and orientation of IMU
        //   SetInitialPose(config_file, *vo);

        //   return vo;
        // }

        ChannelFrameRgbdFisheye::Ptr makeTripleWithDepth(const std::string config_file,
                                                         const std::string calib_file, const CameraBundlePtr& cam)
        {
            CameraBundle::Ptr ncam = (cam) ? cam : loadCameraFromYaml(calib_file);
            if (ncam->numCameras() > 3)
            {
                LOG(WARNING) << "Load more cameras than needed, will erase from the end.";
                ncam->keepFirstNCams(3);
            }

            InitializationOptions init_options = loadInitializationOptions(config_file);
            // cameras: one rgbd camera & two fisheye camera
            // init_options.init_type = InitializerType::kMono;
            ChannelFrameRgbdFisheye::Ptr vo =
                std::make_shared<ChannelFrameRgbdFisheye>(
                    loadBaseOptions(config_file, true),
                    loadDepthOptimizationOptions(config_file),
                    loadDetectorOptions(config_file),
                    init_options,
                    loadReprojectorOptions(config_file),
                    loadTrackerOptions(config_file),
                    ncam);

            // Get initial position and orientation of IMU
            SetInitialPose(config_file, *vo);

            return vo;
        }

    } // namespace factory
} // namespace mivins
