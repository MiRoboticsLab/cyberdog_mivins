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
#include "visual_base_factor.h"
#include "common/parameters.h"

class DiffBundleDiffCamFactor : public VisualBaseFactor, public ceres::SizedCostFunction<2, 7, 7, 7, 7, 1, 1>
{
public:
  DiffBundleDiffCamFactor(const Eigen::Vector3d &pts_i, const Eigen::Vector3d &pts_j,
                          const Eigen::Vector3d &vel_i, const Eigen::Vector3d &vel_j,
                          const double td_i = 0.0f, const double td_j = 0.0f,
                          const int pyr_i = 0, const int pyr_j = 0);

  DiffBundleDiffCamFactor(const Eigen::Vector3d &pts_i, const Eigen::Vector3d &pts_j,
                          const double td_i = 0.0f, const double td_j = 0.0f,
                          const int pyr_i = 0, const int pyr_j = 0);

  void setInitExtrinsicParam(const Eigen::Matrix3d &ric_i, const Eigen::Vector3d &tic_i,
                             const Eigen::Matrix3d &ric_j, const Eigen::Vector3d &tic_j);

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
  
  void check(double **parameters);

  static double sum_t_;
};


class DiffBundleDiffCamFactorP : public VisualBaseFactor, public ceres::SizedCostFunction<2, 7, 7, 1>
{
public:
  DiffBundleDiffCamFactorP(const Eigen::Vector3d &pts_i, const Eigen::Vector3d &pts_j,
                          const int pyr_i = 0, const int pyr_j = 0);

  void setInitExtrinsicParam(const Eigen::Matrix3d &ric_i, const Eigen::Vector3d &tic_i,
                             const Eigen::Matrix3d &ric_j, const Eigen::Vector3d &tic_j);

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

  static double sum_t_;
};
