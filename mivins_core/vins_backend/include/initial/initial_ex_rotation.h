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

#pragma once 

#include <vector>

#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Dense>

#include "common/parameters.h"

/* This class help you to calibrate extrinsic rotation between imu and camera when your totally don't konw the extrinsic parameter */
class InitialEXRotation
{
public:
  InitialEXRotation();
  bool CalibrationExRotation(std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres, Eigen::Quaterniond delta_q_imu, Eigen::Matrix3d &calib_ric_result);
private:
  Eigen::Matrix3d solveRelativeR(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &corres);

  double testTriangulation(const std::vector<cv::Point2f> &l,
               const std::vector<cv::Point2f> &r,
               cv::Mat_<double> R, cv::Mat_<double> t);
  void decomposeE(cv::Mat E,
          cv::Mat_<double> &R1, cv::Mat_<double> &R2,
          cv::Mat_<double> &t1, cv::Mat_<double> &t2);
  int frame_count;

  std::vector< Eigen::Matrix3d > Rc;
  std::vector< Eigen::Matrix3d > Rimu;
  std::vector< Eigen::Matrix3d > Rc_g;
  Eigen::Matrix3d ric;
};


