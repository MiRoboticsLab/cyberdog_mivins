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

#include "common/parameters.h"
#include "common/common_lib.h"

#include <ceres/ceres.h>

class IntegrationBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<IntegrationBase>;

  IntegrationBase() = delete;

  IntegrationBase(const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr,
          const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg);
  
  ~IntegrationBase();

  void setWeight(float weight);

  void pushBack(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr, const bool process = true);
  
  void repropagate(const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg);
  
  void midPointIntegration(double dt, 
            const Eigen::Vector3d &acc_i, const Eigen::Vector3d &gyr_i, 
            const Eigen::Vector3d &acc_j, const Eigen::Vector3d &gyr_j,
            const Eigen::Vector3d &ba, const Eigen::Vector3d &bg, 
            Eigen::Vector3d &delta_p_i, Eigen::Quaterniond &delta_q_i, 
            Eigen::Vector3d &delta_v_i, bool update_jacobian);
  
  void propagate(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr);

  Eigen::Matrix<double, 15, 1> evaluate(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, 
              const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
              const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj, 
              const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj);
  
private:
  void dataCheck(const Eigen::Vector3d &acc_i, const Eigen::Vector3d &gyr_i,
                 const Eigen::Vector3d &acc_j, const Eigen::Vector3d &gyr_j);

public:

  Eigen::Vector3d acc_prv_, gyr_prv_;

  const Eigen::Vector3d linearized_acc_, linearized_gyr_;
  Eigen::Vector3d linearized_ba_, linearized_bg_;

  Eigen::Matrix<double, 15, 15> jacobian_, covariance_;
  Eigen::Matrix<double, 15, 15> step_jacobian_;
  Eigen::Matrix<double, 15, 18> step_V_;
  Eigen::Matrix<double, 18, 18> noise_;

  double sum_dt_;

  double sigma_acc_;
  double sigma_gyr_;

  Eigen::Vector3d delta_p_;
  Eigen::Vector3d delta_v_;
  Eigen::Quaterniond delta_q_;

  std::vector<double> dt_buf_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> acc_buf_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> gyr_buf_;

};

using IntegrationBasePtr = std::shared_ptr<IntegrationBase>; 

