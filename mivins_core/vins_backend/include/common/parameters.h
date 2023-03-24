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

#pragma once

#include <map>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

struct VinsMotionDetectorOptions
{
  double px_diff_threshold = 0.5; ///< maximum allowed pixel distance [px]
  /// (!) ratio of allowed moving correspondences (moving_corresp/total_corresp)
  double ratio_moving_pixels_threshold = 0.1;
  int min_number_correspondences = 5; ///< min number of correspondendces
  int max_features_to_check = 100; ///< max number of correspondences to check
  double sigma = 0.05; ///< standard deviation for return motion prior
};


struct VinsBackendOptions
{
  bool debug = false;
  bool backend_opt = true;
  bool use_edgelet = true;
  bool marg_in_thread = false;
  bool multiple_thread = false;
  bool backend_with_imu = true;
  bool backend_with_odom = true;
  bool backend_with_plane = false;
  bool get_motion_prior_with_imu = true;
  bool get_motion_prior_with_odom = true;
  bool get_motion_prior_with_odompose = false;

  bool opt_full = true;

  bool opt_kf_only = true;

  bool use_loose_couple = false;

  int error_type = 0;

  int window_size = 10;

  int obs_threshold = 2;

  int quality_min_fts = 60;

  int grain_size = 1;

  int kf_new_opt_size = 0;
  int kf_all_opt_size = 0;
  int nkf_all_opt_size = 0;

  int backend_grid_width = 100;
  int backend_grid_height = 100;

  bool estimate_imu_td = false;
  bool estimate_cam_extrinsic = false;

  bool estimate_odom_s3d = false;
  bool estimate_odom_extrinsic = false;

  float min_depth = -1.0f;
  float max_depth = -1.0f;

  float plane_dist = 0.0f;

  // optimization parameters
  float max_solver_time = 0.04;     // max solver itration time (ms), to guarantee real time
  float max_num_iterations = 8;     // max solver itrations, to guarantee real time
  float keyframe_parallax = 10.0f;  // keyframe selection threshold (pixel)

  std::string output_folder;

};

enum SolverFlag
{
  INITIAL,
  NON_LINEAR
};

enum MargFlag
{
  MARG_OLD = 0,
  MARG_SECOND_NEW = 1,
  MARG_FIRST_NEW = 2
};

enum class PipType {
  kRgbd,
  kMono,
  kStereo,
  kTripleWithStereo,
  kTripleWithDepth,
  kStereoWithTwoFisheye
};

enum SIZE_PARAMETERIZATION
{
  SIZE_POSE = 7,
  SIZE_SPEEDBIAS = 9,
  SIZE_DEPTH = 1,

  SIZE_SPEED = 3,
  SIZE_BIAS = 6,

  SIZE_TD = 1
};

enum StateOrder
{
  O_P = 0,
  O_R = 3,
  O_V = 6,
  O_BA = 9,
  O_BG = 12
};

enum NoiseOrder
{
  O_AN = 0,
  O_GN = 3,
  O_AW = 6,
  O_GW = 9
};

enum ErrorType 
{
    kUnitPlane, 
    kSpherePlane,
};

extern int WINDOW_SIZE;
const int NUM_OF_F = 10000;
extern bool ESTIMATE_IMU_TD;
extern bool ESTIMATE_CAM_EXTRINSIC;

extern bool ESTIMATE_ODOM_TD;
extern bool ESTIMATE_ODOM_S3D;
extern bool ESTIMATE_ODOM_EXTRINSIC;


// #define UNIT_SPHERE_ERROR

extern double FOCAL_LENGTH;
extern double INIT_DEPTH;
extern double MIN_PARALLAX;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern Eigen::Vector3d G;

extern Eigen::Matrix3d RIO;
extern Eigen::Vector3d TIO;

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;

extern double ODOM_VEL_N;
extern double ODOM_GYR_N;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;

extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern std::string OUTPUT_FOLDER;
extern double TD_IMU;
extern double TD_ODOM;
extern int ROLLING_SHUTTER;
extern int MULTIPLE_THREAD;

extern bool DEBUG;
extern bool BACKEND_OPT;
extern bool USE_EDGELET;
extern bool BACKEND_WITH_IMU;
extern bool BACKEND_WITH_ODOM;
extern bool USE_BEARING_VECTOR;

extern ErrorType ERROR_TYPE;
extern PipType PIPLINE_TYPE;

void setParameters(const VinsBackendOptions &vins_backend_options, const int pipeline_type);




