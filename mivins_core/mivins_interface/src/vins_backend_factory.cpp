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

#include <svo/vins_backend_factory.h>

#include <memory>
#include <svo/param.h>
// #include <mivins/channel_imu.h>
// #include <mivins/channel_odom.h>

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <yaml-cpp/yaml.h>
#pragma diagnostic pop

namespace mivins
{
    namespace vins_backend_factory
    {

        VinsMotionDetectorOptions loadMotionDetectorOptions(const std::string config_file)
        {
            VinsMotionDetectorOptions o;

            YAML::Node config = YAML::LoadFile(config_file);
            o.px_diff_threshold =
                getParam<double>(config, "zero_motion_px_diff_threshold", 0.5);
            o.ratio_moving_pixels_threshold =
                getParam<double>(config, "zero_motion_ratio_moving_pixels_threshold", 0.1);
            o.min_number_correspondences =
                getParam<int>(config, "zero_motion_min_number_correspondences", 5);
            o.max_features_to_check =
                getParam<int>(config, "zero_motion_max_features_to_check", 100);
            //DEBUG_CHECK(o.max_features_to_check>0) << "max_features_to_check must be > 0";
            o.sigma = getParam<double>(config, "zero_motion_sigma", 0.05);
            return o;
        }

        VinsBackendOptions loadVinsBackendOptions(const std::string config_file)
        {
            VinsBackendOptions o;
            YAML::Node config = YAML::LoadFile(config_file);

            o.debug = getParam<bool>(config, "debug", false);

            o.backend_opt = getParam<bool>(config, "backend_opt", true);

            o.backend_with_imu = getParam<bool>(config, "backend_with_imu", true);

            o.backend_with_odom = getParam<bool>(config, "backend_with_odom", true);

            o.backend_with_plane = getParam<bool>(config, "backend_with_plane", false);

            o.get_motion_prior_with_imu = getParam<bool>(config, "get_motion_prior_with_imu", true);
            
            o.get_motion_prior_with_odom = getParam<bool>(config, "get_motion_prior_with_odom", true);

            o.get_motion_prior_with_odompose = getParam<bool>(config, "get_motion_prior_with_odompose", true);

            o.use_edgelet = getParam<bool>(config, "use_edgelet", true);

            o.opt_full = getParam<bool>(config, "opt_full", true);

            o.opt_kf_only = getParam<bool>(config, "opt_kf_only", true);

            o.marg_in_thread = getParam<bool>(config, "marg_in_thread", false);

            o.multiple_thread = getParam<bool>(config, "multiple_thread", false);

            o.error_type = getParam<int>(config, "error_type", 0);

            o.min_depth = getParam<float>(config, "min_depth", -1.0f);

            o.max_depth = getParam<float>(config, "max_depth", -1.0f);

            o.plane_dist = getParam<float>(config, "plane_dist", 0.0f);

            o.window_size = getParam<int>(config, "window_size", 10);

            o.obs_threshold = getParam<int>(config, "obs_threshold", 2);

            o.grain_size = getParam<int>(config, "grain_size", 1);

            o.kf_new_opt_size = getParam<int>(config, "kf_new_opt_size", 0);

            o.kf_all_opt_size = getParam<int>(config, "kf_all_opt_size", 0);

            o.nkf_all_opt_size = getParam<int>(config, "nkf_all_opt_size", 0);

            o.backend_grid_width = getParam<int>(config, "backend_grid_width", 100);

            o.backend_grid_height = getParam<int>(config, "backend_grid_height", 100);

            o.quality_min_fts = getParam<int>(config, "quality_min_fts", 60);

            o.estimate_cam_extrinsic = getParam<bool>(config, "estimate_cam_extrinsic", false);
            
            o.estimate_odom_s3d = getParam<bool>(config, "estimate_odom_s3d", false);

            o.estimate_odom_extrinsic = getParam<bool>(config, "estimate_odom_extrinsic", false);

            o.use_loose_couple = getParam<bool>(config, "use_loose_couple", false);

            o.max_solver_time = getParam<float>(config, "max_solver_time", 0.04);
            o.max_num_iterations = getParam<int>(config, "max_num_iterations", 8);
            o.keyframe_parallax = getParam<float>(config, "keyframe_parallax", 10.0);

            // o.td = getParam<float>(config, "td", 0.0);
            o.estimate_imu_td = getParam<bool>(config, "estimate_imu_td", false);

            std::cout << "###############################################\n";

            std::cout << "Backend parameters:\n";
            std::cout << "debug: " << o.debug << "\n";
            std::cout << "backend_opt: " << o.backend_opt << "\n";
            std::cout << "opt_full: " << o.opt_full << "\n";
            std::cout << "opt_kf_only: " << o.opt_kf_only << "\n";
            std::cout << "marg_in_thread: " << o.marg_in_thread << "\n";
            std::cout << "mutiple_thread: " << o.multiple_thread << "\n";
            std::cout << "backend_with_imu: " << o.backend_with_imu << "\n";
            std::cout << "error_type: " << o.error_type << "\n";
            
            std::cout << "min_depth: " << o.min_depth << "\n";
            std::cout << "max_depth: " << o.max_depth << "\n";

            std::cout << "plane_dist: " << o.plane_dist << "\n";

            std::cout << "grain_size: " << o.grain_size << "\n";
            std::cout << "kf_new_opt_size: " << o.kf_new_opt_size << "\n";
            std::cout << "kf_all_opt_size: " << o.kf_all_opt_size << "\n";
            std::cout << "nkf_all_opt_size: " << o.nkf_all_opt_size << "\n";

            std::cout << "backend_grid_width: " << o.backend_grid_width << "\n";
            std::cout << "backend_grid_height: " << o.backend_grid_height << "\n";

            std::cout << "window_size: " << o.window_size << "\n";
            std::cout << "obs_threshold: " << o.obs_threshold << "\n";
            std::cout << "estimate_cam_extrinsic: " << o.estimate_cam_extrinsic << "\n";

            std::cout << "max_solver_time: " << o.max_solver_time << "\n";
            std::cout << "max_num_iterations: " << o.max_num_iterations << "\n";
            std::cout << "keyframe_parallax: " << o.keyframe_parallax << "\n";

            std::cout << "backend_with_imu: " << o.backend_with_imu << "\n";
            std::cout << "backend_with_odom: " << o.backend_with_odom << "\n";
            std::cout << "backend_with_plane: " << o.backend_with_plane << "\n";

            std::cout << "get_motion_prior_with_imu: " << o.get_motion_prior_with_imu << "\n";
            std::cout << "get_motion_prior_with_odom: " << o.get_motion_prior_with_odom << "\n";
            std::cout << "get_motion_prior_with_odompose: " << o.get_motion_prior_with_odompose << "\n";
            
            std::cout << "estimate_imu_td: " << o.estimate_imu_td << "\n";

            std::cout << "use_loose_couple: " << o.use_loose_couple << "\n";

            std::cout << "###############################################\n";
            
            return o;
        }

        VinsBackendInterface::Ptr makeBackend(const std::string config_file,
                                              const CameraBundlePtr &camera_bundle,
                                              const int cam_size)
        {
            VinsBackendInterface::Ptr ba_interface =
                std::make_shared<VinsBackendInterface>(
                                                       loadVinsBackendOptions(config_file),
                                                       loadMotionDetectorOptions(config_file),
                                                       camera_bundle, cam_size);

            //   if(getParam<bool>(config, "ba_parallelized", true))
            //     ba_interface->StartFilterThread();

            return ba_interface;
        }

    } // namespace vin_factory
} // namespace mivins
