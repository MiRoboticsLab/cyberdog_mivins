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

#include "common/parameters.h"
#include "common/common_lib.h"

#include <ceres/ceres.h>

#define OdomStateSize 12 + 3
#define OdomNoiseSize 12 + 3

class IntegrationOdom
{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<IntegrationBase>;

  IntegrationOdom() = delete;

  IntegrationOdom(const Eigen::Quaterniond &rot, const Eigen::Vector3d &pos,
                  const Eigen::Vector3d &vel, const Eigen::Vector3d &gyr, 
                  const Eigen::Vector3d &vel_scale, const Eigen::Vector3d &gyr_scale, 
                  const double td = 0.0f, const bool scale_3dof = true);

  ~IntegrationOdom();

  void setWeight(float weight);

  void pushBack(const double dt, 
                const Eigen::Quaterniond &rot, const Eigen::Vector3d &pos,
                const Eigen::Vector3d &vel, const Eigen::Vector3d &gyr);

  void repropagate(const Eigen::Vector3d &linearized_bg);

  void propagate(const double dt, const Eigen::Vector3d &vel, 
                 const Eigen::Vector3d &gyr);

  void midPointIntegrationOdom(const double dt, 
    const Eigen::Vector3d &vel_prv, const Eigen::Vector3d &gyr_prv,
    const Eigen::Vector3d &vel_cur, const Eigen::Vector3d &gyr_cur,
    const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q,
    Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q,
    bool update_jacobian);

  Eigen::Matrix<double, 6, 1> evaluate(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, 
                                       const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj,
                                       const Eigen::Vector3d &vel_scale, 
                                       const Eigen::Vector3d &gyr_scale,
                                       const double td);

  Eigen::Matrix<double, 6, 1> evaluate(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, 
                                       const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj,
                                       const double &vel_scale, const double &gyr_scale,
                                       const double td);

  Eigen::Matrix<double, 6, 1> evaluate(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, 
                                       const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj);

  Eigen::Matrix<double, OdomNoiseSize, OdomNoiseSize> noise_;
  Eigen::Matrix<double, OdomStateSize, OdomStateSize> jacobian_;
  Eigen::Matrix<double, OdomStateSize, OdomStateSize> covariance_;

  Eigen::Quaterniond rot_cur_;
  Eigen::Vector3d pos_cur_;

  Eigen::Vector3d vel_prv_, gyr_prv_;
  Eigen::Vector3d vel_cur_, gyr_cur_;

  double sum_dt_ = 0.0;
  double delta_td_ = 0.0f;
  Eigen::Vector3d delta_p_;
  Eigen::Quaterniond delta_q_;
  Eigen::Vector3d corrected_delta_p_;
  Eigen::Quaterniond corrected_delta_q_;

  double linearized_td_;

  Eigen::Quaterniond linearized_rot_;
  Eigen::Vector3d linearized_pos_;

  Eigen::Vector3d linearized_vel_;
  Eigen::Vector3d linearized_gyr_;

  Eigen::Vector3d linearized_bg_;

  Eigen::Vector3d linearized_vel_scale_;
  Eigen::Vector3d linearized_gyr_scale_;

  bool scale_3dof_ = false;

  std::vector<double> dt_buf_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> vel_buf_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> gyr_buf_;
};

using IntegrationOdomPtr = std::shared_ptr<IntegrationOdom>; 
