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

#include "factor/visual_base_factor.h"

std::vector<Eigen::Matrix2d, Eigen::aligned_allocator<Eigen::Matrix2d>> VisualBaseFactor::sqrt_infos_;

VisualBaseFactor::VisualBaseFactor()
{
  td_ = 0.0f;
  td_i_ = 0.0f;
  td_j_ = 0.0f;
  
  inv_dep_i_ = 0.0f;

  Pi_.setZero();
  Pj_.setZero();
  
  Qi_.setIdentity();
  Qj_.setIdentity();
  
  tic_i_.setZero();
  tic_j_.setZero();
  
  qic_i_.setIdentity();
  qic_j_.setIdentity();

  vel_i_.setZero();
  vel_j_.setZero();

  pts_i_td_.setZero();
  pts_j_td_.setZero();
}

VisualBaseFactor::~VisualBaseFactor()
{

}

void VisualBaseFactor::setErrorInfo(const int pyr_level, 
    const double scale_factor, const double error)
{
  for(int i = 0; i < pyr_level; ++i)
  {
    // double level_sigma = std::pow(scale_factor, i);
    // double level_sigma2 = std::pow(level_sigma, 2);
    // double level_sigma2_inv = 1.0f / level_sigma2 * error;
    double weight = 1.0f / std::sqrt(i + 1);
    Eigen::Matrix2d sqrt_info = weight * error * Eigen::Matrix2d::Identity();
    sqrt_infos_.emplace_back(sqrt_info);
  }
}

void VisualBaseFactor::getTangentBase()
{
  Eigen::Vector3d b1, b2;
  Eigen::Vector3d a = pts_j_.normalized();
  Eigen::Vector3d tmp(0, 0, 1);
  if(a == tmp)
    tmp << 1, 0, 0;
  b1 = (tmp - a * (a.transpose() * tmp)).normalized();
  b2 = a.cross(b1);
  tangent_base_.block<1, 3>(0, 0) = b1.transpose();
  tangent_base_.block<1, 3>(1, 0) = b2.transpose(); 
}

void VisualBaseFactor::setTd(double const *parameters) const
{
  VisualBaseFactor* const fake = const_cast<VisualBaseFactor* const>(this);
  fake->td_ = parameters[0];
}

void VisualBaseFactor::setInvDepth(double const *parameters) const
{
  VisualBaseFactor* const fake = const_cast<VisualBaseFactor* const>(this);
  fake->inv_dep_i_ = parameters[0];
}

void VisualBaseFactor::setImuPose(double const *parameters, int cam_idx) const
{
  VisualBaseFactor* const fake = const_cast<VisualBaseFactor* const>(this);

  if(cam_idx == 0)
  {
    fake->Pi_ = Eigen::Map<Eigen::Vector3d>((double (&)[3])(*(parameters)));
    fake->Qi_ = Eigen::Map<Eigen::Quaterniond>((double (&)[4])(*(parameters + 3)));
  }
  else
  {
    fake->Pj_ = Eigen::Map<Eigen::Vector3d>((double (&)[3])(*(parameters)));
    fake->Qj_ = Eigen::Map<Eigen::Quaterniond>((double (&)[4])(*(parameters + 3)));
  }
}

void VisualBaseFactor::setExtPose(double const *parameters, int cam_idx) const
{
  VisualBaseFactor* const fake = const_cast<VisualBaseFactor* const>(this);

  if(cam_idx == 0)
  {
    fake->tic_i_ = Eigen::Map<Eigen::Vector3d>((double (&)[3])(*(parameters)));
    fake->qic_i_ = Eigen::Map<Eigen::Quaterniond>((double (&)[4])(*(parameters + 3)));
  }
  else
  {
    fake->tic_j_ = Eigen::Map<Eigen::Vector3d>((double (&)[3])(*(parameters)));
    fake->qic_j_ = Eigen::Map<Eigen::Quaterniond>((double (&)[4])(*(parameters + 3)));
  }
}

void VisualBaseFactor::featureProjection(
    Eigen::Vector3d &pts_imu_i, Eigen::Vector3d &pts_imu_j,
    Eigen::Vector3d &pts_cam_i, Eigen::Vector3d &pts_cam_j)const
{
  VisualBaseFactor* const fake = const_cast<VisualBaseFactor* const>(this);
  fake->pts_i_td_ = pts_i_ - (td_ - td_i_) * vel_i_;
  fake->pts_j_td_ = pts_j_ - (td_ - td_j_) * vel_j_;
  
  pts_cam_i = pts_i_td_ / inv_dep_i_;
  pts_imu_i = qic_i_ * pts_cam_i + tic_i_;
  Eigen::Vector3d pts_world = Qi_ * pts_imu_i + Pi_;
  pts_imu_j = Qj_.inverse() * (pts_world - Pj_);
  pts_cam_j = qic_j_ .inverse() * (pts_imu_j - tic_j_);
}


Eigen::Vector2d VisualBaseFactor::computeResidual(
                const Eigen::Vector3d &pts_obs, 
                const Eigen::Vector3d &pts_cal) const
{
  Eigen::Vector2d residual;

  if(ERROR_TYPE == ErrorType::kSpherePlane)
    residual =  tangent_base_ * (pts_cal.normalized() - pts_obs.normalized());
  else if(ERROR_TYPE == ErrorType::kUnitPlane)
    residual = (pts_cal / pts_cal.z()).head<2>() - pts_obs.head<2>();

  residual = sqrt_info_ * residual;

  return residual;
}

Eigen::Matrix<double, 2, 3> VisualBaseFactor::computeJacobian(
                const Eigen::Vector3d pts_cam) const
{
  Eigen::Matrix<double, 2, 3> reduce;

  if(ERROR_TYPE == ErrorType::kSpherePlane)
  {
    double x = pts_cam.x();
    double y = pts_cam.y();
    double z = pts_cam.z();
    double norm = pts_cam.norm();
    double norm3 = std::pow(norm, 3);

    Eigen::Matrix3d norm_jaco;
    norm_jaco << 1.0/norm - x*x/norm3, -x*y/norm3,           -x*z/norm3,
                -x*y/norm3,            1.0/norm - y*y/norm3, -y*z/norm3,
                -x*z/norm3,            -y*z/norm3,            1.0/norm - z*z/norm3;

    reduce = tangent_base_ * norm_jaco;
  }
  else if(ERROR_TYPE == ErrorType::kUnitPlane)
  {
    double x = pts_cam.x();
    double y = pts_cam.y();
    double z = pts_cam.z();
    reduce << 1.0/z, 0.0, -x/(z*z),
              0.0, 1.0/z, -y/(z*z);
  }

  reduce = sqrt_info_ * reduce;

  return reduce;
}
