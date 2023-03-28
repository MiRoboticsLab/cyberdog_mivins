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

#include "common/parameters.h"

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

Eigen::Matrix3d RIO;
Eigen::Vector3d TIO;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double ODOM_VEL_N;
double ODOM_GYR_N;

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;

bool ESTIMATE_IMU_TD;
bool ESTIMATE_CAM_EXTRINSIC;

bool ESTIMATE_ODOM_TD;
bool ESTIMATE_ODOM_S3D;
bool ESTIMATE_ODOM_EXTRINSIC;

int ROLLING_SHUTTER;
// std::string VINS_RESULT_PATH;
std::string OUTPUT_FOLDER;

double TD_IMU;
double TD_ODOM;
int MULTIPLE_THREAD;

bool DEBUG;
bool BACKEND_OPT;
bool USE_EDGELET;
bool BACKEND_WITH_IMU;
bool BACKEND_WITH_ODOM;
bool USE_BEARING_VECTOR = false;

ErrorType ERROR_TYPE;
PipType PIPLINE_TYPE;
int WINDOW_SIZE = 10;
double FOCAL_LENGTH = 460.0;

void setParameters(const VinsBackendOptions &vins_backend_options, const int pipeline_type)
{
  PIPLINE_TYPE = PipType(pipeline_type);

  DEBUG = vins_backend_options.debug;

  USE_EDGELET = vins_backend_options.use_edgelet;

  WINDOW_SIZE = vins_backend_options.window_size;
  
  BACKEND_OPT = vins_backend_options.backend_opt;
  
  MULTIPLE_THREAD = vins_backend_options.multiple_thread;

  BACKEND_WITH_IMU = vins_backend_options.backend_with_imu; 

  BACKEND_WITH_ODOM = vins_backend_options.backend_with_odom; 

  MIN_PARALLAX = vins_backend_options.keyframe_parallax;

  int error_type = vins_backend_options.error_type;
  if(error_type == 0)
    ERROR_TYPE = ErrorType::kUnitPlane;
  else if(error_type == 1)
    ERROR_TYPE = ErrorType::kSpherePlane;
  else
  {
    std::cout << "Error: bad error type, please check config file\n";
    exit(0);
  }

  SOLVER_TIME = vins_backend_options.max_solver_time;
  NUM_ITERATIONS = vins_backend_options.max_num_iterations;
  OUTPUT_FOLDER = vins_backend_options.output_folder;
  
  ESTIMATE_CAM_EXTRINSIC = vins_backend_options.estimate_cam_extrinsic;
  // if (ESTIMATE_CAM_EXTRINSIC == 2)
  // {
  //   printf("have no prior about extrinsic param, calibrate extrinsic param\n");
  //   RIC.push_back(Eigen::Matrix3d::Identity());
  //   TIC.push_back(Eigen::Vector3d::Zero());
  // }
  // else 
  // {
  //   if ( ESTIMATE_CAM_EXTRINSIC == 1)
  //   {
  //     printf(" Optimize extrinsic param around initial guess!\n");
  //   }
  //   if (ESTIMATE_CAM_EXTRINSIC == 0)
  //     printf(" fix extrinsic param \n");
  // } 

  ESTIMATE_ODOM_S3D = vins_backend_options.estimate_odom_s3d;
  ESTIMATE_ODOM_EXTRINSIC = vins_backend_options.estimate_odom_extrinsic;

  INIT_DEPTH = 5.0;
  BIAS_ACC_THRESHOLD = 0.1;
  BIAS_GYR_THRESHOLD = 0.1;

  // TD_IMU = vins_backend_options.td;
  ESTIMATE_IMU_TD = vins_backend_options.estimate_imu_td;

  switch (PIPLINE_TYPE)
  {
    case PipType::kRgbd:
      std::cout << "PipelineType: kRgbd\n";
      break;
    case PipType::kMono:
      std::cout << "PipelineType: kMono\n";
      break;
    case PipType::kStereo:
      std::cout << "PipelineType: kStereo\n";
      break;
    case PipType::kTripleWithStereo:
      std::cout << "PipelineType: kTripleWithStereo\n";
      break;
    case PipType::kTripleWithDepth:
      std::cout << "PipelineType: kTripleWithDepth\n";
      break;
    case PipType::kStereoWithTwoFisheye:
      std::cout << "PipelineType: kStereoWithTwoFisheye\n";
      break;

    default:
      std::cout << "Unknown pipeline\n";
      break;
  }
}
