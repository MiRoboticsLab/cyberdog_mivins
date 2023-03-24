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

#include "factor/integration_odom.h"

IntegrationOdom::IntegrationOdom(
    const Eigen::Quaterniond &rot, const Eigen::Vector3d &pos,
    const Eigen::Vector3d &vel, const Eigen::Vector3d &gyr, 
    const Eigen::Vector3d &vel_scale, const Eigen::Vector3d &gyr_scale, 
    const double td, const bool scale_3dof)
{
  scale_3dof_ = scale_3dof;

  vel_prv_ = vel;
  gyr_prv_ = gyr;

  linearized_td_ = td;

  linearized_rot_ = rot;
  linearized_pos_ = pos;

  linearized_vel_ = vel;
  linearized_gyr_ = gyr;

  linearized_vel_scale_ = vel_scale;
  linearized_gyr_scale_ = gyr_scale;

  sum_dt_ = 0.0f;
  delta_p_ = Eigen::Vector3d::Zero();
  delta_q_ = Eigen::Quaterniond::Identity();

  covariance_ = Eigen::Matrix<double, OdomStateSize, OdomStateSize>::Zero();
  jacobian_ = Eigen::Matrix<double, OdomStateSize, OdomStateSize>::Identity();

  Eigen::Matrix3d I3x3 = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d vel_noise = std::pow(ODOM_VEL_N, 2) * I3x3;
  Eigen::Matrix3d gyr_noise = std::pow(ODOM_GYR_N, 2) * I3x3;
  Eigen::Matrix3d bg_noise = std::pow(0.001, 2) * I3x3;

  noise_ = Eigen::Matrix<double, OdomNoiseSize, OdomNoiseSize>::Zero();
  noise_.block<3, 3>(0, 0) = vel_noise;
  noise_.block<3, 3>(3, 3) = gyr_noise;
  noise_.block<3, 3>(6, 6) = vel_noise;
  noise_.block<3, 3>(9, 9) = gyr_noise;
  noise_.block<3, 3>(12, 12) = bg_noise;

  linearized_bg_ = Eigen::Vector3d(1e-5, 1e-5, 1e-5);
}

IntegrationOdom::~IntegrationOdom()
{
  dt_buf_.clear();
  vel_buf_.clear();
  gyr_buf_.clear();
}

void IntegrationOdom::setWeight(float weight)
{
  assert(weight >= 0);

  noise_ *= weight;
}

void IntegrationOdom::pushBack(const double dt, 
  const Eigen::Quaterniond &rot, const Eigen::Vector3d &pos, 
  const Eigen::Vector3d &vel, const Eigen::Vector3d &gyr)
{
  rot_cur_ = rot;
  pos_cur_ = pos;

  dt_buf_.emplace_back(dt);
  vel_buf_.emplace_back(vel);
  gyr_buf_.emplace_back(gyr);

  propagate(dt, vel, gyr);
}

void IntegrationOdom::repropagate(const Eigen::Vector3d &linearized_bg)
{
  sum_dt_ = 0.0f;

  vel_prv_ = linearized_vel_;
  gyr_prv_ = linearized_gyr_;
  linearized_bg_ = linearized_bg;

  delta_p_.setZero();
  delta_q_.setIdentity();

  jacobian_.setIdentity();
  covariance_.setZero();

  for (int i = 0; i < static_cast<int>(dt_buf_.size()); i++)
    propagate(dt_buf_[i], vel_buf_[i], gyr_buf_[i]);
}

void IntegrationOdom::propagate(const double dt, 
  const Eigen::Vector3d &vel, const Eigen::Vector3d &gyr)
{
  vel_cur_ = vel;
  gyr_cur_ = gyr;

  Eigen::Vector3d result_delta_p = delta_p_;
  Eigen::Quaterniond result_delta_q = delta_q_;

  midPointIntegrationOdom(dt, vel_prv_, gyr_prv_, vel_cur_, gyr_cur_,
              delta_p_, delta_q_, result_delta_p, result_delta_q, true);

  sum_dt_ += dt;

  delta_p_ = result_delta_p;
  delta_q_ = result_delta_q;
  vel_prv_ = vel_cur_;
  gyr_prv_ = gyr_cur_;
}

void IntegrationOdom::midPointIntegrationOdom(const double dt, 
  const Eigen::Vector3d &vel_prv, const Eigen::Vector3d &gyr_prv,
  const Eigen::Vector3d &vel_cur, const Eigen::Vector3d &gyr_cur,
  const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q,
  Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q,
  bool update_jacobian)
{  
  Eigen::Matrix3d vs = linearized_vel_scale_.asDiagonal();
  Eigen::Matrix3d gs = linearized_gyr_scale_.asDiagonal();
  
  Eigen::Vector3d un_gyr = 0.5f * gs * (gyr_prv + gyr_cur) - linearized_bg_;
  result_delta_q = delta_q * Utility::deltaQ(un_gyr * dt).normalized();

  Eigen::Vector3d vel_0 = delta_q * vs * vel_prv;
  Eigen::Vector3d vel_1 = result_delta_q * vs * vel_cur;
  Eigen::Vector3d un_vel = 0.5f * (vel_0 + vel_1);
  result_delta_p = delta_p + un_vel * dt;

  if(update_jacobian)
  {
    Eigen::Matrix3d I3x3 = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d I_x = Eigen::Vector3d::UnitX().asDiagonal();
    Eigen::Matrix3d I_y = Eigen::Vector3d::UnitY().asDiagonal();
    Eigen::Matrix3d I_z = Eigen::Vector3d::UnitZ().asDiagonal();

    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(OdomStateSize, OdomStateSize);
    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(OdomStateSize, OdomNoiseSize);

    Eigen::Matrix3d gyr_hat = Utility::skewSymmetric(un_gyr);
    Eigen::Matrix3d vel_hat_0 = Utility::skewSymmetric(vs * vel_prv);
    Eigen::Matrix3d vel_hat_1 = Utility::skewSymmetric(vs * vel_cur);

    F.block<3, 3>(0, 0) = I3x3;
    F.block<3, 3>(0, 3) = -0.5 * dt * (delta_q.toRotationMatrix() * vel_hat_0
          + result_delta_q.toRotationMatrix() * vel_hat_1 * (I3x3 - gyr_hat * dt));
    F.block<3, 3>(0, 6) = 0.5 * dt * result_delta_q.toRotationMatrix() * vel_hat_1 * gs;


    F.block<3, 3>(3, 3) = I3x3 - gyr_hat * dt;
    F.block<3, 3>(3, 6) = -dt * gs;

    F.block<3, 3>(6, 6) = I3x3;

    if(scale_3dof_)
    {
      F.block<3, 1>(0, 9) = 0.5 * dt * (delta_q.toRotationMatrix() * I_x * vel_prv 
                              + result_delta_q.toRotationMatrix() * I_x * vel_cur);
      F.block<3, 1>(0, 10) = 0.5 * dt * (delta_q.toRotationMatrix() * I_y * vel_prv 
                              + result_delta_q.toRotationMatrix() * I_y * vel_cur);
      F.block<3, 1>(0, 11) = 0.5 * dt * (delta_q.toRotationMatrix() * I_z * vel_prv 
                              + result_delta_q.toRotationMatrix() * I_z * vel_cur);
                              
      F.block<3, 1>(0, 12) = -0.25 * dt * dt * result_delta_q.toRotationMatrix() * vel_hat_1 
                              * I_x * (gyr_prv - linearized_bg_ + gyr_cur - linearized_bg_);
      F.block<3, 1>(0, 13) = -0.25 * dt * dt * result_delta_q.toRotationMatrix() * vel_hat_1 
                              * I_y * (gyr_prv - linearized_bg_ + gyr_cur - linearized_bg_);
      F.block<3, 1>(0, 14) = -0.25 * dt * dt * result_delta_q.toRotationMatrix() * vel_hat_1 
                              * I_z * (gyr_prv - linearized_bg_ + gyr_cur - linearized_bg_);

      F.block<3, 3>(3, 12) = (0.5 * dt * (gyr_prv - linearized_bg_ + gyr_cur - linearized_bg_)).asDiagonal();

      F.block<3, 3>(9, 9) = I3x3;
      F.block<3, 3>(12, 12) = I3x3;
    }
    else
    {
      F.block<3, 1>(0, 9) = 0.5 * dt * (delta_q.toRotationMatrix() * vel_prv 
                              + result_delta_q.toRotationMatrix() * vel_cur);
      F.block<3, 1>(0, 10)  = -0.25 * dt * dt * result_delta_q.toRotationMatrix() * vel_hat_1 
                              * (gyr_prv - linearized_bg_ + gyr_cur - linearized_bg_);
      
      F.block<3, 1>(3, 10) = 0.5 * dt * (gyr_prv - linearized_bg_ + gyr_cur - linearized_bg_);

      F(9, 9) = 1.0;
      F(10, 10) = 1.0;
    }

    V.block<3, 3>(0, 0) = -0.5 * dt * delta_q.toRotationMatrix() * vs;
    V.block<3, 3>(0, 3) = 0.25 * dt * dt * result_delta_q.toRotationMatrix() * vel_hat_1 * gs;    
    V.block<3, 3>(0, 6) = -0.5 * dt * result_delta_q.toRotationMatrix() * vs;
    V.block<3, 3>(0, 9) = 0.25 * dt * dt * result_delta_q.toRotationMatrix() * vel_hat_1 * gs;
    V.block<3, 3>(3, 3) = -0.5 * dt * gs;
    V.block<3, 3>(3, 9) = -0.5 * dt * gs;
    V.block<3, 3>(6, 12) = dt * I3x3;
    V.block<3, 3>(9, 9) = dt * vs * I3x3;
    V.block<3, 3>(12, 12) = dt * gs * I3x3;

    jacobian_ = F * jacobian_;
    covariance_ = F * covariance_ * F.transpose() + V * noise_ * V.transpose();
  }
}

Eigen::Matrix<double, 6, 1> IntegrationOdom::evaluate(
    const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, 
    const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj,
    const Eigen::Vector3d &vel_scale, const Eigen::Vector3d &gyr_scale,
    const double td)
{
  Eigen::Matrix<double, 6, 1> residuals;

  Eigen::Vector3d dp_dvsx = jacobian_.block<3, 1>(0, 9);
  Eigen::Vector3d dp_dvsy = jacobian_.block<3, 1>(0, 10);
  Eigen::Vector3d dp_dvsz = jacobian_.block<3, 1>(0, 11);

  Eigen::Vector3d dq_dgsx = jacobian_.block<3, 1>(3, 12);
  Eigen::Vector3d dq_dgsy = jacobian_.block<3, 1>(3, 13);
  Eigen::Vector3d dq_dgsz = jacobian_.block<3, 1>(3, 14);

  Eigen::Vector3d delta_vel_scale = vel_scale - linearized_vel_scale_;
  Eigen::Vector3d delta_gyr_scale = gyr_scale - linearized_gyr_scale_;

  Eigen::Matrix3d vs = vel_scale.asDiagonal();
  Eigen::Matrix3d gs = gyr_scale.asDiagonal();

  delta_td_ = td - linearized_td_;

  corrected_delta_q_ = delta_q_ * Utility::deltaQ(dq_dgsx * delta_gyr_scale.x() +
                        dq_dgsy * delta_gyr_scale.y() + dq_dgsz * delta_gyr_scale.z()).normalized();

  corrected_delta_p_ = delta_p_ + dp_dvsx * delta_vel_scale.x() 
                        + dp_dvsy * delta_vel_scale.y() + dp_dvsz * delta_vel_scale.z();

  Eigen::Quaterniond delta_q = Utility::deltaQ(linearized_gyr_ * delta_td_).normalized() 
              * corrected_delta_q_ * Utility::deltaQ(-gyr_cur_ * delta_td_).normalized();

  Eigen::Vector3d delta_p = Utility::deltaQ(linearized_gyr_ * delta_td_).normalized()
              * (vs * linearized_vel_ * delta_td_ + corrected_delta_p_ 
                  - corrected_delta_q_ * vs * vel_cur_ * delta_td_);

  residuals.block<3, 1>(0, 0) = Qi.inverse() * (Pj - Pi) - delta_p_;
  residuals.block<3, 1>(3, 0) = 2 * (delta_q_.inverse() * (Qi.inverse() * Qj)).vec();

  return residuals;
}

Eigen::Matrix<double, 6, 1> IntegrationOdom::evaluate(
    const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, 
    const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj,
    const double &vel_scale, const double &gyr_scale,
    const double td)
{
  Eigen::Matrix<double, 6, 1> residuals;

  Eigen::Vector3d dp_dvs = jacobian_.block<3, 1>(0, 9);
  Eigen::Vector3d dp_dgs = jacobian_.block<3, 1>(0, 10);
  Eigen::Vector3d dq_dgs = jacobian_.block<3, 1>(3, 9);
  
  delta_td_ = td - linearized_td_;

  double delta_vs = vel_scale - linearized_vel_scale_.x();
  double delta_gs = gyr_scale - linearized_gyr_scale_.x();

  corrected_delta_p_ = delta_p_ + dp_dvs * delta_vs + dp_dgs * delta_gs;
  corrected_delta_q_ = delta_q_ * Utility::deltaQ(dq_dgs * delta_gs).normalized();;

  Eigen::Matrix3d vs = vel_scale * Eigen::Matrix3d::Identity();
  Eigen::Matrix3d gs = gyr_scale * Eigen::Matrix3d::Identity();

  Eigen::Quaterniond delta_q = Utility::deltaQ(gs * linearized_gyr_ * delta_td_).normalized() 
              * corrected_delta_q_ * Utility::deltaQ(-gs * gyr_cur_ * delta_td_).normalized();

  Eigen::Vector3d delta_p = Utility::deltaQ(gs * linearized_gyr_ * delta_td_).normalized()
              * (vs * linearized_vel_ * delta_td_ + corrected_delta_p_ 
                  - corrected_delta_q_ * vs * vel_cur_ * delta_td_);

  residuals.block<3, 1>(0, 0) = Qi.inverse() * (Pj - Pi) - delta_p_;
  residuals.block<3, 1>(3, 0) = 2 * (delta_q_.inverse() * (Qi.inverse() * Qj)).vec();

  return residuals;
}

Eigen::Matrix<double, 6, 1> IntegrationOdom::evaluate(
      const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, 
      const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj)
{
  Eigen::Matrix<double, 6, 1> residuals;

  // Eigen::Matrix3d dp_dbg = jacobian_.block<3, 3>(0, 6);
  // Eigen::Matrix3d dq_dbg = jacobian_.block<3, 3>(3, 6);
  // Eigen::Vector3d dbg = Bgi - linearized_bg_;

  Eigen::Quaterniond corrected_delta_q = delta_q_; // * Utility::deltaQ(dq_dbg * dbg);
  Eigen::Vector3d corrected_delta_p = delta_p_; // + dp_dbg * dbg;

  residuals.block<3, 1>(0, 0) = Qi.inverse() * (Pj - Pi) - corrected_delta_p;
  residuals.block<3, 1>(3, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
  // residuals.block<3, 1>(6, 0) = Bgj - Bgi;

  return residuals;
}
