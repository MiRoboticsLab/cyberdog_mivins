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

#include "factor/same_bundle_diff_cam_factor.h"
#include "common/common_lib.h"

double SameBundleDiffCamFactor::sum_t_;

SameBundleDiffCamFactor::SameBundleDiffCamFactor(
    const Eigen::Vector3d &pts_i, const Eigen::Vector3d &pts_j,
    const Eigen::Vector3d &vel_i, const Eigen::Vector3d &vel_j,
    const double td_i, const double td_j,
    const int pyr_i, const int pyr_j) 
{
  td_i_ = td_i;
  td_j_ = td_j;

  pyr_i_ = pyr_i;
  pyr_j_ = pyr_j;

  pts_i_ = pts_i;
  pts_j_ = pts_j; 

  vel_i_ = vel_i;
  vel_j_ = vel_j;

  int pyr_level = std::max(pyr_i, pyr_j);
  sqrt_info_ = sqrt_infos_[pyr_level];

  if(ERROR_TYPE == ErrorType::kSpherePlane)
    getTangentBase();
};

SameBundleDiffCamFactor::SameBundleDiffCamFactor(
    const Eigen::Vector3d &pts_i, const Eigen::Vector3d &pts_j,
    const double td_i, const double td_j,
    const int pyr_i, const int pyr_j) 
{
  td_i_ = td_i;
  td_j_ = td_j;

  pyr_i_ = pyr_i;
  pyr_j_ = pyr_j;

  pts_i_ = pts_i;
  pts_j_ = pts_j; 

  int pyr_level = std::max(pyr_i, pyr_j);
  sqrt_info_ = sqrt_infos_[pyr_level];

  if(ERROR_TYPE == ErrorType::kSpherePlane)
    getTangentBase();
};

void SameBundleDiffCamFactor::setInitExtrinsicParam(
    const Eigen::Matrix3d &ric_i, const Eigen::Vector3d &tic_i,
    const Eigen::Matrix3d &ric_j, const Eigen::Vector3d &tic_j)
{
  qic_i_ = ric_i;
  tic_i_ = tic_i;

  qic_j_ = ric_j;
  tic_j_ = tic_j;
}

bool SameBundleDiffCamFactor::Evaluate(
        double const *const *parameters, 
        double *residuals, double **jacobians) const
{
  Timer tim;
  tim.tic("evaluate");

  setExtPose(parameters[0], 0);

  setExtPose(parameters[1], 1);

  setInvDepth(parameters[2]);

  setTd(parameters[3]);

  Eigen::Vector3d pts_imu_i, pts_imu_j, pts_cam_i, pts_cam_j;
  featureProjection(pts_imu_i, pts_imu_j, pts_cam_i, pts_cam_j);
  
  Eigen::Map<Eigen::Vector2d> residual(residuals);
  residual = computeResidual(pts_j_td_, pts_cam_j);

  if (jacobians)
  {
    Eigen::Matrix3d ric_i = qic_i_.toRotationMatrix();
    Eigen::Matrix3d ric_j = qic_j_.toRotationMatrix();

    Eigen::Matrix<double, 2, 3> reduce(2, 3);
    reduce = computeJacobian(pts_cam_j);

    if (jacobians[0])
    {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[0]);
      jacobian_ex_pose.setZero();

      Eigen::Matrix<double, 3, 6> jaco_ex;
      jaco_ex.leftCols<3>() = ric_j.transpose(); 
      jaco_ex.rightCols<3>() = ric_j.transpose() * ric_i * -Utility::skewSymmetric(pts_cam_i);
      
      jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
    }
    if (jacobians[1])
    {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose1(jacobians[1]);
      jacobian_ex_pose1.setZero();

      Eigen::Matrix<double, 3, 6> jaco_ex;
      jaco_ex.leftCols<3>() = - ric_j.transpose();
      jaco_ex.rightCols<3>() = Utility::skewSymmetric(pts_cam_j);
      
      jacobian_ex_pose1.leftCols<6>() = reduce * jaco_ex;
    }
    if (jacobians[2])
    {
      Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[2]);
#if 1
      jacobian_feature = reduce * ric_j.transpose() * ric_i * pts_i_ * -(1.0 / (inv_dep_i_ * inv_dep_i_));
#else
      jacobian_feature = reduce * ric_i.transpose() * Rj.transpose() * Ri * ric_i * pts_i_;
#endif
    }
    if (jacobians[3])
    {
      Eigen::Map<Eigen::Vector2d> jacobian_td(jacobians[3]);
      jacobian_td = reduce * ric_j.transpose() * ric_i * vel_i_ / inv_dep_i_ * -1.0  
                    + sqrt_info_ * vel_j_.head(2);
    }
  }
  sum_t_ += tim.toc("evaluate");

  return true;
}

void SameBundleDiffCamFactor::check(double **parameters)
{
  double *res = new double[15];
  double **jaco = new double *[4];
  jaco[0] = new double[2 * 7];
  jaco[1] = new double[2 * 7];
  jaco[2] = new double[2 * 1];
  jaco[3] = new double[2 * 1];
  Evaluate(parameters, res, jaco);
  puts("check begins");

  puts("my");

  std::cout << Eigen::Map<Eigen::Matrix<double, 2, 1>>(res).transpose() << std::endl
        << std::endl;
  std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[0]) << std::endl
        << std::endl;
  std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[1]) << std::endl
        << std::endl;
  std::cout << Eigen::Map<Eigen::Vector2d>(jaco[2]) << std::endl
        << std::endl;
  std::cout << Eigen::Map<Eigen::Vector2d>(jaco[3]) << std::endl
        << std::endl;

  Eigen::Vector3d tic_i(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond qic_i(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

  Eigen::Vector3d tic_j(parameters[1][0], parameters[1][1], parameters[1][2]);
  Eigen::Quaterniond qic_j(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

  double inv_dep_i = parameters[2][0];

  double td = parameters[3][0];

  Eigen::Vector3d pts_i_td, pts_j_td;
  pts_i_td = pts_i_ - (td - td_i_) * vel_i_;
  pts_j_td = pts_j_ - (td - td_j_) * vel_j_;

  Eigen::Vector3d pts_cam_i = pts_i_td / inv_dep_i;
  Eigen::Vector3d pts_imu_i = qic_i * pts_cam_i + tic_i;
  Eigen::Vector3d pts_imu_j = pts_imu_i;
  Eigen::Vector3d pts_cam_j = qic_j.inverse() * (pts_imu_j - tic_j);


  Eigen::Vector2d residual;

  if(ERROR_TYPE == ErrorType::kSpherePlane)
    residual =  tangent_base_ * (pts_cam_j.normalized() - pts_j_td.normalized());
  else if(ERROR_TYPE == ErrorType::kUnitPlane)
  {
    double dep_j = pts_cam_j.z();
    residual = (pts_cam_j / dep_j).head<2>() - pts_j_td.head<2>();
  }

  residual = sqrt_info_ * residual;

  puts("num");
  std::cout << residual.transpose() << std::endl;

  const double eps = 1e-6;
  Eigen::Matrix<double, 2, 14> num_jacobian;
  for (int k = 0; k < 14; k++)
  {
    Eigen::Vector3d tic_i(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond qic_i(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d tic_j(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond qic_j(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    double inv_dep_i = parameters[2][0];

    double td = parameters[3][0];

    int a = k / 3, b = k % 3;
    Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

    if (a == 0)
      tic_i += delta;
    else if (a == 1)
      qic_i = qic_i * Utility::deltaQ(delta);
    else if (a == 2)
      tic_j += delta;
    else if (a == 3)
      qic_j = qic_j * Utility::deltaQ(delta);
    else if (a == 4)
    {
      if (b == 0)
        inv_dep_i += delta.x();
      else
        td += delta.y();
    }

    Eigen::Vector3d pts_i_td, pts_j_td;
    pts_i_td = pts_i_ - (td - td_i_) * vel_i_;
    pts_j_td = pts_j_ - (td - td_j_) * vel_j_;

    Eigen::Vector3d pts_cam_i = pts_i_td / inv_dep_i;
    Eigen::Vector3d pts_imu_i = qic_i * pts_cam_i + tic_i;
    Eigen::Vector3d pts_imu_j = pts_imu_i;
    Eigen::Vector3d pts_cam_j = qic_j.inverse() * (pts_imu_j - tic_j);

    Eigen::Vector2d tmp_residual;

    if(ERROR_TYPE == ErrorType::kSpherePlane)
      tmp_residual =  tangent_base_ * (pts_cam_j.normalized() - pts_j_td.normalized());
    else if(ERROR_TYPE == ErrorType::kUnitPlane)
    {
      double dep_j = pts_cam_j.z();
      tmp_residual = (pts_cam_j / dep_j).head<2>() - pts_j_td.head<2>();
    }

    tmp_residual = sqrt_info_ * tmp_residual;
    num_jacobian.col(k) = (tmp_residual - residual) / eps;
  }
  std::cout << num_jacobian.block<2, 6>(0, 0) << std::endl;
  std::cout << num_jacobian.block<2, 6>(0, 6) << std::endl;
  std::cout << num_jacobian.block<2, 1>(0, 12) << std::endl;
  std::cout << num_jacobian.block<2, 1>(0, 13) << std::endl;
}


/*****************************************************************************************/
SameBundleDiffCamFactorP::SameBundleDiffCamFactorP(
    const Eigen::Vector3d &pts_i, const Eigen::Vector3d &pts_j,
    const int pyr_i, const int pyr_j) 
{
  pyr_i_ = pyr_i;
  pyr_j_ = pyr_j;

  pts_i_ = pts_i;
  pts_j_ = pts_j; 

  int pyr_level = std::max(pyr_i, pyr_j);
  sqrt_info_ = sqrt_infos_[pyr_level];

  if(ERROR_TYPE == ErrorType::kSpherePlane)
    getTangentBase();
};

void SameBundleDiffCamFactorP::setInitExtrinsicParam(
    const Eigen::Matrix3d &ric_i, const Eigen::Vector3d &tic_i,
    const Eigen::Matrix3d &ric_j, const Eigen::Vector3d &tic_j)
{
  qic_i_ = ric_i;
  tic_i_ = tic_i;

  qic_j_ = ric_j;
  tic_j_ = tic_j;
}

bool SameBundleDiffCamFactorP::Evaluate(
        double const *const *parameters, 
        double *residuals, double **jacobians) const
{
  setInvDepth(parameters[0]);

  Eigen::Vector3d pts_imu_i, pts_imu_j, pts_cam_i, pts_cam_j;
  featureProjection(pts_imu_i, pts_imu_j, pts_cam_i, pts_cam_j);
  
  Eigen::Map<Eigen::Vector2d> residual(residuals);
  residual = computeResidual(pts_j_td_, pts_cam_j);

  double inv_inv_dep_i2 = 1.0 / (inv_dep_i_ * inv_dep_i_);

  if (jacobians)
  {
    Eigen::Matrix3d ric_i = qic_i_.toRotationMatrix();
    Eigen::Matrix3d ric_j = qic_j_.toRotationMatrix();

    Eigen::Matrix<double, 2, 3> reduce(2, 3);
    reduce = computeJacobian(pts_cam_j);

    if (jacobians[0])
    {
      Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[0]);
      jacobian_feature.setZero();
      
#if 1
      jacobian_feature = - reduce * ric_j.transpose() * ric_i * pts_i_ * inv_inv_dep_i2;
#else
      jacobian_feature = reduce * ric_i.transpose() * Rj.transpose() * Ri * ric_i * pts_i_;
#endif
    }
  }

  return true;
}
