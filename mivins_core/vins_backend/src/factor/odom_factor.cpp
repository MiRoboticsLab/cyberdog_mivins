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

#include "factor/odom_factor.h"

OdomFactor::OdomFactor(IntegrationOdomPtr pre_integration)
  :pre_integration_(pre_integration)
{

}

OdomFactor::~OdomFactor()
{

}

bool OdomFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
  Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

  Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
  Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

  Eigen::Vector3d tio(parameters[2][0], parameters[2][1], parameters[2][2]);
  Eigen::Quaterniond qio(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

  // Eigen::Vector3d Bgi(parameters[3][0], parameters[3][1], parameters[3][2]);

  // Eigen::Vector3d Bgj(parameters[4][0], parameters[4][1], parameters[4][2]);

  Eigen::Quaterniond Q_w_oi = Qi * qio;
  Eigen::Vector3d P_w_oi = Qi * tio + Pi;  
  Eigen::Quaterniond Q_w_oj = Qj * qio;
  Eigen::Vector3d P_w_oj = Qj * tio + Pj;

  Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
  residual = pre_integration_->evaluate(P_w_oi, Q_w_oi, P_w_oj, Q_w_oj);

  Eigen::Matrix<double, 6, 6> covariance = 
        pre_integration_->covariance_.block<6, 6>(0, 0);
  Eigen::Matrix<double, 6, 6> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 6, 6>>(
        covariance.inverse()).matrixL().transpose();

  residual = sqrt_info * residual;

  if(jacobians)
  {
    if(jacobians[0])
    {
      Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
      jacobian_pose_i.setZero();

      jacobian_pose_i.block<3, 3>(0, 0) = -(Qi * qio).inverse().toRotationMatrix();
      jacobian_pose_i.block<3, 3>(0, 3) = qio.inverse().toRotationMatrix() 
        * Utility::skewSymmetric(Qi.inverse().toRotationMatrix() * (Pj - Pi + Qj * tio));
      jacobian_pose_i.block<3, 3>(3, 3) = -(Utility::Qright(Qi.inverse() * Qj * qio) 
        * Utility::Qleft(pre_integration_->delta_q_.inverse() * qio.inverse())).bottomRightCorner<3, 3>();

      jacobian_pose_i = sqrt_info * jacobian_pose_i;
    }
    if(jacobians[1])
    {
      Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);
      jacobian_pose_j.setZero();

      jacobian_pose_j.block<3, 3>(0, 0) = (Qi * qio).inverse().toRotationMatrix();
      jacobian_pose_j.block<3, 3>(0, 3) = -(qio.inverse() * Qi.inverse() * Qj).toRotationMatrix() 
        * Utility::skewSymmetric(tio);
      jacobian_pose_j.block<3, 3>(3, 3) = (Utility::Qright(qio) 
        * Utility::Qleft(pre_integration_->delta_q_.inverse() * qio.inverse() * Qi.inverse() * Qj)).bottomRightCorner<3, 3>();

      jacobian_pose_j = sqrt_info * jacobian_pose_j;
    }
    if(jacobians[2])
    {
      Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_ex(jacobians[2]);
      jacobian_ex.setZero();

      jacobian_ex.block<3, 3>(0, 0) = (qio.inverse() * Qi.inverse() * Qj).toRotationMatrix() - qio.inverse().toRotationMatrix();
      jacobian_ex.block<3, 3>(0, 3) = Utility::skewSymmetric(qio.inverse() * Qi.inverse() * (Pj - Pi + Qj * tio))
                      - Utility::skewSymmetric(qio.inverse() * tio);
      jacobian_ex.block<3, 3>(3, 3) = 2 * Utility::Qleft(pre_integration_->delta_q_.inverse()).bottomRightCorner<3, 3>()
                      * Utility::skewSymmetric( (qio.inverse() * Qi.inverse() * Qj * qio).vec() );

      jacobian_ex = sqrt_info * jacobian_ex;
    }
    // if(jacobians[3])
    // {
    //   Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jacobian_i(jacobians[3]);
    //   jacobian_i.setZero();
    //   jacobian_i.block<3, 3>(0, 0) = -pre_integration_->jacobian_.template block<3, 3>(0, 6);
    //   jacobian_i.block<3, 3>(3, 0) = -Utility::Qleft(Qj.inverse() * Qi * pre_integration_->delta_q_).bottomRightCorner<3, 3>() 
    //                                   * pre_integration_->jacobian_.template block<3, 3>(3, 6);
    //   jacobian_i.block<3, 3>(6, 0) = -Eigen::Matrix3d::Identity();

    //   jacobian_i = sqrt_info * jacobian_i;
    // }
    // if(jacobians[4])
    // {
    //   Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jacobian_j(jacobians[4]);
    //   jacobian_j.setZero();

    //   jacobian_j.block<3, 3>(6, 0) = Eigen::Matrix3d::Identity();

    //   jacobian_j = sqrt_info * jacobian_j;
    // }
  }

  return true;
}
