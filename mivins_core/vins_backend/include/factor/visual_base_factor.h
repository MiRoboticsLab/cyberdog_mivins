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

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include "common/parameters.h"

class VisualBaseFactor
{
public:

  VisualBaseFactor();

  ~VisualBaseFactor();

  // no copy
  VisualBaseFactor(const VisualBaseFactor&) = delete;
  VisualBaseFactor& operator=(const VisualBaseFactor&) = delete;

  void getTangentBase();

  void setTd(double const *parameters) const;

  void setInvDepth(double const *parameters) const;

  void setImuPose(double const *parameters, int cam_idx) const;

  void setExtPose(double const *parameters, int cam_idx) const;

  void featureProjection(
      Eigen::Vector3d &pts_imu_i, Eigen::Vector3d &pts_imu_j,
      Eigen::Vector3d &pts_cam_i, Eigen::Vector3d &pts_cam_j)const;

  Eigen::Vector2d computeResidual(const Eigen::Vector3d &pts_obs, 
                                  const Eigen::Vector3d &pts_cal) const;

  Eigen::Matrix<double, 2, 3> computeJacobian(const Eigen::Vector3d pts_cam) const;

  static void setErrorInfo(const int pyr_level, const double scale_factor, const double error);

public:
  int pyr_i_, pyr_j_;
  double td_i_, td_j_;
  Eigen::Vector3d pts_i_, pts_j_;
  Eigen::Vector3d vel_i_, vel_j_;
  Eigen::Vector3d pts_i_td_, pts_j_td_;
  
  Eigen::Matrix<double, 2, 3> tangent_base_;

  Eigen::Matrix2d sqrt_info_;
  static std::vector<Eigen::Matrix2d, Eigen::aligned_allocator<Eigen::Matrix2d>> sqrt_infos_;
  

  double td_;
  double inv_dep_i_;

  Eigen::Vector3d Pi_, Pj_;
  Eigen::Quaterniond Qi_, Qj_;
  
  Eigen::Vector3d tic_i_, tic_j_;
  Eigen::Quaterniond qic_i_, qic_j_;

EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
