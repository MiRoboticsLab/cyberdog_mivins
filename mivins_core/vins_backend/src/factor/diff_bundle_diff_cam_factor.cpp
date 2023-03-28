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

#include "factor/diff_bundle_diff_cam_factor.h"
#include "common/common_lib.h"

double DiffBundleDiffCamFactor::sum_t_;

DiffBundleDiffCamFactor::DiffBundleDiffCamFactor(
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

DiffBundleDiffCamFactor::DiffBundleDiffCamFactor(
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

void DiffBundleDiffCamFactor::setInitExtrinsicParam(
    const Eigen::Matrix3d &ric_i, const Eigen::Vector3d &tic_i,
    const Eigen::Matrix3d &ric_j, const Eigen::Vector3d &tic_j)
{
  qic_i_ = ric_i;
  tic_i_ = tic_i;

  qic_j_ = ric_j;
  tic_j_ = tic_j;
}


bool DiffBundleDiffCamFactor::Evaluate(
        double const *const *parameters, 
        double *residuals, double **jacobians) const
{
  Timer tim;
  tim.tic("evaluate");

  setImuPose(parameters[0], 0);

  setImuPose(parameters[1], 1);

  setExtPose(parameters[2], 0);

  setExtPose(parameters[3], 1);

  setInvDepth(parameters[4]);

  setTd(parameters[5]);

  Eigen::Vector3d pts_imu_i, pts_imu_j, pts_cam_i, pts_cam_j;
  featureProjection(pts_imu_i, pts_imu_j, pts_cam_i, pts_cam_j);

  Eigen::Map<Eigen::Vector2d> residual(residuals);
  residual = computeResidual(pts_j_td_, pts_cam_j);

  if (jacobians)
  {
    Eigen::Matrix3d Ri = Qi_.toRotationMatrix();
    Eigen::Matrix3d Rj = Qj_.toRotationMatrix();
    Eigen::Matrix3d ric_i = qic_i_.toRotationMatrix();
    Eigen::Matrix3d ric_j = qic_j_.toRotationMatrix();
    Eigen::Matrix<double, 2, 3> reduce(2, 3);

    reduce = computeJacobian(pts_cam_j);

    if (jacobians[0])
    {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
      jacobian_pose_i.setZero();

      Eigen::Matrix<double, 3, 6> jaco_i;
      jaco_i.leftCols<3>() = ric_j.transpose() * Rj.transpose();
      jaco_i.rightCols<3>() = ric_j.transpose() * Rj.transpose() * Ri * -Utility::skewSymmetric(pts_imu_i);

      jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
    }

    if (jacobians[1])
    {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);
      jacobian_pose_j.setZero();

      Eigen::Matrix<double, 3, 6> jaco_j;
      jaco_j.leftCols<3>() = ric_j.transpose() * -Rj.transpose();
      jaco_j.rightCols<3>() = ric_j.transpose() * Utility::skewSymmetric(pts_imu_j);

      jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
    }
    if (jacobians[2])
    {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[2]);
      jacobian_ex_pose.setZero();

      Eigen::Matrix<double, 3, 6> jaco_ex;
      jaco_ex.leftCols<3>() = ric_j.transpose() * Rj.transpose() * Ri; 
      jaco_ex.rightCols<3>() = ric_j.transpose() * Rj.transpose() * Ri * ric_i * -Utility::skewSymmetric(pts_cam_i);
      
      jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
    }
    if (jacobians[3])
    {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose1(jacobians[3]);
      jacobian_ex_pose1.setZero();

      Eigen::Matrix<double, 3, 6> jaco_ex;
      jaco_ex.leftCols<3>() = - ric_j.transpose();
      jaco_ex.rightCols<3>() = Utility::skewSymmetric(pts_cam_j);
      
      jacobian_ex_pose1.leftCols<6>() = reduce * jaco_ex;
    }
    if (jacobians[4])
    {
      Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[4]);
#if 1
      jacobian_feature = reduce * ric_j.transpose() * Rj.transpose() * Ri * ric_i * pts_i_td_ * -(1.0 / (inv_dep_i_ * inv_dep_i_));
#else
      jacobian_feature = reduce * ric_i.transpose() * Rj.transpose() * Ri * ric_i * pts_i_;
#endif
    }
    if (jacobians[5])
    {
      Eigen::Map<Eigen::Vector2d> jacobian_td(jacobians[5]);
      jacobian_td = reduce * ric_j.transpose() * Rj.transpose() * Ri * ric_i * vel_i_ / inv_dep_i_ * -1.0  +
              sqrt_info_ * vel_j_.head(2);
    }
  }
  sum_t_ += tim.toc("evaluate");

  return true;
}

void DiffBundleDiffCamFactor::check(double **parameters)
{
  double *res = new double[15];
  double **jaco = new double *[6];
  jaco[0] = new double[2 * 7];
  jaco[1] = new double[2 * 7];
  jaco[2] = new double[2 * 7];
  jaco[3] = new double[2 * 7];
  jaco[4] = new double[2 * 1];
  jaco[5] = new double[2 * 1];
  Evaluate(parameters, res, jaco);
  puts("check begins");

  puts("my");

  std::cout << Eigen::Map<Eigen::Matrix<double, 2, 1>>(res).transpose() << std::endl
        << std::endl;
  std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[0]) << std::endl
        << std::endl;
  std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[1]) << std::endl
        << std::endl;
  std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[2]) << std::endl
        << std::endl;
  std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[3]) << std::endl
        << std::endl;
  std::cout << Eigen::Map<Eigen::Vector2d>(jaco[4]) << std::endl
        << std::endl;
  std::cout << Eigen::Map<Eigen::Vector2d>(jaco[5]) << std::endl
        << std::endl;

  Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

  Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
  Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

  Eigen::Vector3d tic_i(parameters[2][0], parameters[2][1], parameters[2][2]);
  Eigen::Quaterniond qic_i(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

  Eigen::Vector3d tic_j(parameters[3][0], parameters[3][1], parameters[3][2]);
  Eigen::Quaterniond qic_j(parameters[3][6], parameters[3][3], parameters[3][4], parameters[3][5]);

  double inv_dep_i = parameters[4][0];

  double td = parameters[5][0];

  Eigen::Vector3d pts_i_td, pts_j_td;
  pts_i_td = pts_i_ - (td - td_i_) * vel_i_;
  pts_j_td = pts_j_ - (td - td_j_) * vel_j_;

  Eigen::Vector3d pts_cam_i = pts_i_td / inv_dep_i;
  Eigen::Vector3d pts_imu_i = qic_i * pts_cam_i + tic_i;
  Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
  Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
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
  Eigen::Matrix<double, 2, 26> num_jacobian;
  for (int k = 0; k < 26; k++)
  {
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    Eigen::Vector3d tic_i(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond qic_i(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

    Eigen::Vector3d tic_j(parameters[3][0], parameters[3][1], parameters[3][2]);
    Eigen::Quaterniond qic_j(parameters[3][6], parameters[3][3], parameters[3][4], parameters[3][5]);

    double inv_dep_i = parameters[4][0];

    double td = parameters[5][0];

    int a = k / 3, b = k % 3;
    Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

    if (a == 0)
      Pi += delta;
    else if (a == 1)
      Qi = Qi * Utility::deltaQ(delta);
    else if (a == 2)
      Pj += delta;
    else if (a == 3)
      Qj = Qj * Utility::deltaQ(delta);
    else if (a == 4)
      tic_i += delta;
    else if (a == 5)
      qic_i = qic_i * Utility::deltaQ(delta);
    else if (a == 6)
      tic_j += delta;
    else if (a == 7)
      qic_j = qic_j * Utility::deltaQ(delta);
    else if (a == 8)
    {
      if(b == 0)
        inv_dep_i += delta.x();
      else
        td += delta.y();
    }

    Eigen::Vector3d pts_i_td, pts_j_td;
    pts_i_td = pts_i_ - (td - td_i_) * vel_i_;
    pts_j_td = pts_j_ - (td - td_j_) * vel_j_;

    Eigen::Vector3d pts_cam_i = pts_i_td / inv_dep_i;
    Eigen::Vector3d pts_imu_i = qic_i * pts_cam_i + tic_i;
    Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
    Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
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
  std::cout << num_jacobian.block<2, 6>(0, 12) << std::endl;
  std::cout << num_jacobian.block<2, 6>(0, 18) << std::endl;
  std::cout << num_jacobian.block<2, 1>(0, 24) << std::endl;
  std::cout << num_jacobian.block<2, 1>(0, 25) << std::endl;
}

/****************************************************************************************/

DiffBundleDiffCamFactorP::DiffBundleDiffCamFactorP(
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

void DiffBundleDiffCamFactorP::setInitExtrinsicParam(
    const Eigen::Matrix3d &ric_i, const Eigen::Vector3d &tic_i,
    const Eigen::Matrix3d &ric_j, const Eigen::Vector3d &tic_j)
{
  qic_i_ = ric_i;
  tic_i_ = tic_i;

  qic_j_ = ric_j;
  tic_j_ = tic_j;
}

bool DiffBundleDiffCamFactorP::Evaluate(
        double const *const *parameters, 
        double *residuals, double **jacobians) const
{
  setImuPose(parameters[0], 0);

  setImuPose(parameters[1], 1);

  setInvDepth(parameters[2]);

  Eigen::Vector3d pts_imu_i, pts_imu_j, pts_cam_i, pts_cam_j;
  featureProjection(pts_imu_i, pts_imu_j, pts_cam_i, pts_cam_j);

  Eigen::Map<Eigen::Vector2d> residual(residuals);
  residual = computeResidual(pts_j_td_, pts_cam_j);

  if (jacobians)
  {
    Eigen::Matrix3d Ri = Qi_.toRotationMatrix();
    Eigen::Matrix3d Rj = Qj_.toRotationMatrix();
    Eigen::Matrix3d ric_i = qic_i_.toRotationMatrix();
    Eigen::Matrix3d ric_j = qic_j_.toRotationMatrix();
    Eigen::Matrix<double, 2, 3> reduce(2, 3);

    reduce = computeJacobian(pts_cam_j);

    double inv_inv_dep_i2 = 1.0 / (inv_dep_i_ * inv_dep_i_);

    Eigen::Matrix3d R_cj_w_tmp = ric_j.transpose() * Rj.transpose();
    Eigen::Matrix<double, 2, 3> tmp = reduce * R_cj_w_tmp;

    if (jacobians[0])
    {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
      jacobian_pose_i.setZero();

      Eigen::Matrix<double, 2, 6> jaco_i;
      jacobian_pose_i.block(0, 0, 2, 3) = tmp;
      jacobian_pose_i.block(0, 3, 2, 3) = - tmp * Ri * Utility::skewSymmetric(pts_imu_i);
    }

    if (jacobians[1])
    {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);
      jacobian_pose_j.setZero();

      Eigen::Matrix<double, 2, 6> jaco_j;
      jacobian_pose_j.block(0, 0, 2, 3) = - tmp;
      jacobian_pose_j.block(0, 3, 2, 3) = reduce * ric_j.transpose() * Utility::skewSymmetric(pts_imu_j);
    }
    if (jacobians[2])
    {
      Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[2]);
      jacobian_feature.setZero();

#if 1
      jacobian_feature = - tmp * Ri * ric_i * pts_i_td_ * inv_inv_dep_i2;
#else
      jacobian_feature = reduce * ric_i.transpose() * Rj.transpose() * Ri * ric_i * pts_i_;
#endif
    }
  }

  return true;
}
