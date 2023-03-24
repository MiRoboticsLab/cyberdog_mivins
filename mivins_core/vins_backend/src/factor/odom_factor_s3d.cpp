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

#include "factor/odom_factor_s3d.h"

OdomFactorS3d::OdomFactorS3d(IntegrationOdomPtr pre_integration)
  :pre_integration_(pre_integration)
{

}

OdomFactorS3d::~OdomFactorS3d()
{

}

bool OdomFactorS3d::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
  Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

  Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
  Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

  Eigen::Vector3d tio(parameters[2][0], parameters[2][1], parameters[2][2]);
  Eigen::Quaterniond qio(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

  Eigen::Vector3d vel_scale;
  vel_scale.x() = parameters[3][0];
  vel_scale.y() = parameters[4][0];
  vel_scale.z() = parameters[5][0];

  Eigen::Vector3d gyr_scale;
  gyr_scale.x() = parameters[6][0];
  gyr_scale.y() = parameters[7][0];
  gyr_scale.z() = parameters[8][0];

  double td = parameters[9][0];

  Eigen::Vector3d P_w_oi = Qi * tio + Pi;  
  Eigen::Quaterniond Q_w_oi = Qi * qio;
  Eigen::Vector3d P_w_oj = Qj * tio + Pj;
  Eigen::Quaterniond Q_w_oj = Qj * qio;

  Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
  residual = pre_integration_->evaluate(P_w_oi, Q_w_oi, P_w_oj, Q_w_oj, vel_scale, gyr_scale, td);

  const Eigen::Matrix<double, 6, 6> &covariance = 
        pre_integration_->covariance_.block<6, 6>(0, 0);
  Eigen::Matrix<double, 6, 6> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 6, 6>>(
        covariance.inverse()).matrixL().transpose();

  residual = sqrt_info * residual;

  // const double &delta_td = pre_integration_->delta_td_;
  // const Eigen::Vector3d &linear_vel_end = pre_integration_->vel_cur_;
  // const Eigen::Vector3d &linear_vel_begin = pre_integration_->linearized_vel_;
  // Eigen::Vector3d p1 = vel_scale.asDiagonal() * linear_vel_end * delta_td;
  // Eigen::Vector3d p0 = linear_vel_scale.asDiagonal() * linear_vel_begin * delta_td;

  if(jacobians)
  {
    Eigen::Vector3d dp_dvsx = pre_integration_->jacobian_.template block<3, 1>(0, 9);
    Eigen::Vector3d dp_dvsy = pre_integration_->jacobian_.template block<3, 1>(0, 10);
    Eigen::Vector3d dp_dvsz = pre_integration_->jacobian_.template block<3, 1>(0, 11);

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
      Eigen::Map<Eigen::Matrix<double, 6, 1>> jacobian_vel_sx(jacobians[3]);
      jacobian_vel_sx.setZero();

      jacobian_vel_sx.block<3, 1>(0, 0) = -dp_dvsx;

      jacobian_vel_sx = sqrt_info * jacobian_vel_sx;
    }
    if(jacobians[4])
    {
      Eigen::Map<Eigen::Matrix<double, 6, 1>> jacobian_vel_sy(jacobians[4]);
      jacobian_vel_sy.setZero();

      jacobian_vel_sy.block<3, 1>(0, 0) = -dp_dvsy;
      
      jacobian_vel_sy = sqrt_info * jacobian_vel_sy;
    }
    if(jacobians[5])
    {
      Eigen::Map<Eigen::Matrix<double, 6, 1>> jacobian_vel_sz(jacobians[5]);
      jacobian_vel_sz.setZero();
      
      jacobian_vel_sz.block<3, 1>(0, 0) = -dp_dvsz;
      
      jacobian_vel_sz = sqrt_info * jacobian_vel_sz;
    }
    if(jacobians[6])
    {
      Eigen::Map<Eigen::Matrix<double, 6, 1>> jacobian_gyr_sx(jacobians[6]);
      jacobian_gyr_sx.setZero();

      Eigen::Vector3d dp_dgsx = pre_integration_->jacobian_.template block<3, 1>(0, 12);
      Eigen::Vector3d dq_dgsx = pre_integration_->jacobian_.template block<3, 1>(3, 12);

      jacobian_gyr_sx.block<3, 1>(0, 0) = -dp_dgsx;
      jacobian_gyr_sx.block<3, 1>(3, 0) = -Utility::Qleft(qio.inverse() * Qj.inverse() 
          * Qi * qio * pre_integration_->delta_q_).bottomRightCorner<3, 3>() * dq_dgsx;

      jacobian_gyr_sx = sqrt_info * jacobian_gyr_sx;
    }
    if(jacobians[7])
    {
      Eigen::Map<Eigen::Matrix<double, 6, 1>> jacobian_gyr_sy(jacobians[7]);
      jacobian_gyr_sy.setZero();

      Eigen::Vector3d dp_dgsy = pre_integration_->jacobian_.template block<3, 1>(0, 13);
      Eigen::Vector3d dq_dgsy = pre_integration_->jacobian_.template block<3, 1>(3, 13);

      jacobian_gyr_sy.block<3, 1>(0, 0) = -dp_dgsy;
      jacobian_gyr_sy.block<3, 1>(3, 0) = -Utility::Qleft(qio.inverse() * Qj.inverse() 
          * Qi * qio * pre_integration_->delta_q_).bottomRightCorner<3, 3>() * dq_dgsy;

      jacobian_gyr_sy = sqrt_info * jacobian_gyr_sy;
    }
    if(jacobians[8])
    {
      Eigen::Map<Eigen::Matrix<double, 6, 1>> jacobian_gyr_sz(jacobians[8]);
      jacobian_gyr_sz.setZero();
      
      Eigen::Vector3d dp_dgsz = pre_integration_->jacobian_.template block<3, 1>(0, 14);
      Eigen::Vector3d dq_dgsz = pre_integration_->jacobian_.template block<3, 1>(3, 14);

      jacobian_gyr_sz.block<3, 1>(0, 0) = -dp_dgsz;
      jacobian_gyr_sz.block<3, 1>(3, 0) = -Utility::Qleft(qio.inverse() * Qj.inverse()
          * Qi * qio * pre_integration_->delta_q_).bottomRightCorner<3, 3>() * dq_dgsz;
      
      jacobian_gyr_sz = sqrt_info * jacobian_gyr_sz;
    }
    
    if(jacobians[9])
    {
      Eigen::Map<Eigen::Matrix<double, 6, 1>> jacobian_td(jacobians[9]);
      jacobian_td.setZero();

      double delta_td = pre_integration_->delta_td_;
      Eigen::Matrix3d sv = vel_scale.asDiagonal();
      Eigen::Vector3d linear_vel_0 = pre_integration_->linearized_vel_;
      Eigen::Vector3d linear_vel_1 = pre_integration_->vel_cur_;

      Eigen::Matrix3d angular_vel_hat_0 = Utility::skewSymmetric(pre_integration_->linearized_gyr_);
      Eigen::Matrix3d angular_vel_hat_1 = Utility::skewSymmetric(pre_integration_->vel_cur_);

      jacobian_td.block<3, 1>(0, 0) = -sv * linear_vel_0
                                      + pre_integration_->corrected_delta_q_ * sv * linear_vel_1
                                      - 2 * angular_vel_hat_1 * sv * linear_vel_0 * delta_td
                                      + angular_vel_hat_1 * pre_integration_->corrected_delta_p_ 
                                      + 2 * angular_vel_hat_1 * pre_integration_->corrected_delta_q_ * sv * linear_vel_1 * delta_td;
      Eigen::Quaterniond q((angular_vel_hat_1 * pre_integration_->corrected_delta_q_.inverse() 
                          - pre_integration_->corrected_delta_q_.inverse() * angular_vel_hat_0
                          - 2 * angular_vel_hat_1 * pre_integration_->corrected_delta_q_.inverse() * angular_vel_hat_0 * delta_td)
                        * pre_integration_->delta_q_);
      jacobian_td.block<3, 1>(3, 0) = 2 * q.vec();

      jacobian_td = sqrt_info * jacobian_td;
    }
    

  }


  return true;
}
