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

#include "factor/odom_factor_s1d.h"

OdomFactorS1d::OdomFactorS1d(IntegrationOdomPtr pre_integration)
  :pre_integration_(pre_integration)
{

}

OdomFactorS1d::~OdomFactorS1d()
{

}

bool OdomFactorS1d::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
  Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

  Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
  Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

  Eigen::Vector3d tio(parameters[2][0], parameters[2][1], parameters[2][2]);
  Eigen::Quaterniond qio(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

  double vel_scale = parameters[3][0];

  double gyr_scale = parameters[4][0];

  double td = parameters[5][0];

  Eigen::Quaterniond Q_w_oi = Qi * qio;
  Eigen::Vector3d P_w_oi = Qi * tio + Pi;  
  Eigen::Quaterniond Q_w_oj = Qj * qio;
  Eigen::Vector3d P_w_oj = Qj * tio + Pj;

  Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
  residual = pre_integration_->evaluate(P_w_oi, Q_w_oi, P_w_oj, Q_w_oj, vel_scale, gyr_scale, td);

  const Eigen::Matrix<double, 6, 6> &covariance = 
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
    
    if(jacobians[3])
    {
      Eigen::Map<Eigen::Matrix<double, 6, 1>> jacobian_vs(jacobians[3]);
      jacobian_vs.setZero();

      Eigen::Vector3d dp_dvs = pre_integration_->jacobian_.template block<3, 1>(0, 9);

      jacobian_vs.block<3, 1>(0, 0) = -dp_dvs;

      jacobian_vs = sqrt_info * jacobian_vs;
    }
    if(jacobians[4])
    {
      Eigen::Map<Eigen::Matrix<double, 6, 1>> jacobian_gs(jacobians[4]);
      jacobian_gs.setZero();

      Eigen::Vector3d dp_dgs = pre_integration_->jacobian_.template block<3, 1>(0, 10);
      Eigen::Vector3d dq_dgs = pre_integration_->jacobian_.template block<3, 1>(3, 10);

      jacobian_gs.block<3, 1>(0, 0) = -dp_dgs;
      jacobian_gs.block<3, 1>(3, 0) = -Utility::Qleft(qio.inverse() * Qj.inverse() 
          * Qi * qio * pre_integration_->delta_q_).bottomRightCorner<3, 3>() * dq_dgs;

      jacobian_gs = sqrt_info * jacobian_gs;
    }
    
    if(jacobians[5])
    {
      Eigen::Map<Eigen::Matrix<double, 6, 1>> jacobian_td(jacobians[5]);
      jacobian_td.setZero();

      double delta_td = pre_integration_->delta_td_;
      Eigen::Matrix3d sv = vel_scale * Eigen::Matrix3d::Identity();
      Eigen::Matrix3d gv = gyr_scale * Eigen::Matrix3d::Identity();
      
      Eigen::Vector3d vel_0 = pre_integration_->linearized_vel_;
      Eigen::Vector3d vel_1 = pre_integration_->vel_cur_;

      Eigen::Matrix3d gyr_hat_0 = Utility::skewSymmetric(pre_integration_->linearized_gyr_);
      Eigen::Matrix3d gyr_hat_1 = Utility::skewSymmetric(pre_integration_->vel_cur_);

      jacobian_td.block<3, 1>(0, 0) = -sv * vel_0
                                      + pre_integration_->corrected_delta_q_ * sv * vel_1
                                      - 2 * gyr_hat_1 * sv * vel_0 * delta_td
                                      - gyr_hat_1 * pre_integration_->corrected_delta_p_ 
                                      + 2 * gyr_hat_1 * pre_integration_->corrected_delta_q_ * sv * vel_1 * delta_td;
      Eigen::Quaterniond q((gyr_hat_1 * pre_integration_->corrected_delta_q_.inverse() 
                          - pre_integration_->corrected_delta_q_.inverse() * gyr_hat_0
                          - 2 * gyr_hat_1 * pre_integration_->corrected_delta_q_.inverse() * gyr_hat_0 * delta_td)
                        * pre_integration_->delta_q_);
      jacobian_td.block<3, 1>(3, 0) = 2 * q.vec();

      jacobian_td = sqrt_info * jacobian_td;
    }
  }

  return true;
}
