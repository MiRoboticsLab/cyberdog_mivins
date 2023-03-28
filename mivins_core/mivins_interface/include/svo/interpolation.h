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

#include <cmath>
#include <cassert>
#include <cstring>
#include <iostream>
#include <eigen3/Eigen/Dense>

namespace loose_couple
{
class Interpolation
{
public:
  Interpolation();
  ~Interpolation();

  bool poseInterpolation(
    const double ts1, const Eigen::Quaterniond &q1, const Eigen::Vector3d &p1, 
    const double ts2, const Eigen::Quaterniond &q2, const Eigen::Vector3d &p2,
    const double ts_inter, Eigen::Quaterniond &q_inter, Eigen::Vector3d &p_inter);


  bool poseInterpolation(const double ts1, const Eigen::Matrix4d &pose1, 
                const double ts2, const Eigen::Matrix4d &pose2, 
                const double ts_inter, Eigen::Matrix4d &pose_inter);

  bool linearInterpolation(double ts1, const Eigen::Vector3d &vel1,
                 double ts2, const Eigen::Vector3d &vel2,
                 double ts_inter, Eigen::Vector3d &vel_inter);

  Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R, bool use_angle);

  Eigen::Matrix3d ypr2R(const Eigen::Vector3d &ypr, bool use_angle);


private:
  Eigen::Quaterniond quatSlerp(Eigen::Quaterniond qa, Eigen::Quaterniond qb, double scalar); 
};
}