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

#include "factor/integration_base.h"


IntegrationBase::IntegrationBase(const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr,
        const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg)
  : acc_prv_{acc}, gyr_prv_{gyr}, linearized_acc_{acc}, linearized_gyr_{gyr},
    linearized_ba_{linearized_ba}, linearized_bg_{linearized_bg}
{
  sum_dt_ = 0.0f;
  sigma_acc_ = 1.0f;
  sigma_gyr_ = 1.0f;

  delta_p_.setZero();
  delta_v_.setZero();
  delta_q_.setIdentity();

  covariance_.setZero();
  jacobian_.setIdentity();

  noise_ = Eigen::Matrix<double, 18, 18>::Zero();
  noise_.block<3, 3>(0, 0)   = (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
  noise_.block<3, 3>(3, 3)   = (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
  noise_.block<3, 3>(6, 6)   = (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
  noise_.block<3, 3>(9, 9)   = (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
  noise_.block<3, 3>(12, 12) = (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
  noise_.block<3, 3>(15, 15) = (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();
}

IntegrationBase::~IntegrationBase()
{
  dt_buf_.clear();
  acc_buf_.clear();
  gyr_buf_.clear();
}

void IntegrationBase::setWeight(float weight)
{
  assert(weight > 0.0f);

  noise_.block<3, 3>(0, 0) *= weight;
  noise_.block<3, 3>(6, 6) *= weight;
  noise_.block<3, 3>(12, 12) *= weight;
}

void IntegrationBase::pushBack(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr, const bool process)
{
  dt_buf_.emplace_back(dt);
  acc_buf_.emplace_back(acc);
  gyr_buf_.emplace_back(gyr);

  if(process)
    propagate(dt, acc, gyr);
}

void IntegrationBase::repropagate(const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg)
{
  sum_dt_ = 0.0f;
  sigma_acc_ = 1.0f;
  sigma_gyr_ = 1.0f;

  acc_prv_ = linearized_acc_;
  gyr_prv_ = linearized_gyr_;
  linearized_ba_ = linearized_ba;
  linearized_bg_ = linearized_bg;

  delta_p_.setZero();
  delta_v_.setZero();
  delta_q_.setIdentity();

  jacobian_.setIdentity();
  covariance_.setZero();

  for (int i = 0; i < static_cast<int>(dt_buf_.size()); i++)
    propagate(dt_buf_[i], acc_buf_[i], gyr_buf_[i]);
}

void IntegrationBase::midPointIntegration(double dt, 
            const Eigen::Vector3d &acc_i, const Eigen::Vector3d &gyr_i,
            const Eigen::Vector3d &acc_j, const Eigen::Vector3d &gyr_j,
            const Eigen::Vector3d &ba, const Eigen::Vector3d &bg, 
            Eigen::Vector3d &delta_p_i, Eigen::Quaterniond &delta_q_i, 
            Eigen::Vector3d &delta_v_i, bool update_jacobian)
{
  Eigen::Vector3d un_gyr = 0.5 * (gyr_i + gyr_j) - bg;
  Eigen::Quaterniond delta_q_j = delta_q_i * Utility::deltaQ(un_gyr * dt);

  Eigen::Vector3d un_acc_i = delta_q_i * (acc_i - ba);
  Eigen::Vector3d un_acc_j = delta_q_j * (acc_j - ba);
  Eigen::Vector3d un_acc = 0.5 * (un_acc_i + un_acc_j);

  Eigen::Vector3d delta_p_j = delta_p_i + delta_v_i * dt + 0.5 * un_acc * dt * dt;
  Eigen::Vector3d delta_v_j = delta_v_i + un_acc * dt;

  if(update_jacobian)
  {
    Eigen::Vector3d w_x = 0.5 * (gyr_i + gyr_j) - bg;
    Eigen::Vector3d a_i_x = acc_i - ba;
    Eigen::Vector3d a_j_x = acc_j - ba;

    Eigen::Matrix3d R_w_x = Utility::skewSymmetric(w_x);
    Eigen::Matrix3d R_a_i_x = Utility::skewSymmetric(a_i_x);
    Eigen::Matrix3d R_a_j_x = Utility::skewSymmetric(a_j_x);

    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(15, 15);
    F.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    F.block<3, 3>(0, 3) = -0.25 * delta_q_i.toRotationMatrix() * R_a_i_x * dt * dt + 
                -0.25 * delta_q_j.toRotationMatrix() * R_a_j_x * (Eigen::Matrix3d::Identity() - R_w_x * dt) * dt * dt;
    F.block<3, 3>(0, 6) = Eigen::MatrixXd::Identity(3,3) * dt;
    F.block<3, 3>(0, 9) = -0.25 * (delta_q_i.toRotationMatrix() + delta_q_j.toRotationMatrix()) * dt * dt;
    F.block<3, 3>(0, 12) = -0.25 * delta_q_j.toRotationMatrix() * R_a_j_x * dt * dt * -dt;
    F.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() - R_w_x * dt;
    F.block<3, 3>(3, 12) = -1.0 * Eigen::MatrixXd::Identity(3,3) * dt;
    F.block<3, 3>(6, 3) = -0.5 * delta_q_i.toRotationMatrix() * R_a_i_x * dt + 
                -0.5 * delta_q_j.toRotationMatrix() * R_a_j_x * (Eigen::Matrix3d::Identity() - R_w_x * dt) * dt;
    F.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
    F.block<3, 3>(6, 9) = -0.5 * (delta_q_i.toRotationMatrix() + delta_q_j.toRotationMatrix()) * dt;
    F.block<3, 3>(6, 12) = -0.5 * delta_q_j.toRotationMatrix() * R_a_j_x * dt * -dt;
    F.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
    F.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity();
    //cout<<"A"<<std::endl<<A<<std::endl;

    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(15,18);
    V.block<3, 3>(0, 0) =  0.25 * delta_q_i.toRotationMatrix() * dt * dt;
    V.block<3, 3>(0, 3) =  0.25 * -delta_q_j.toRotationMatrix() * R_a_j_x  * dt * dt * 0.5 * dt;
    V.block<3, 3>(0, 6) =  0.25 * delta_q_j.toRotationMatrix() * dt * dt;
    V.block<3, 3>(0, 9) =  V.block<3, 3>(0, 3);
    V.block<3, 3>(3, 3) =  0.5 * Eigen::MatrixXd::Identity(3,3) * dt;
    V.block<3, 3>(3, 9) =  0.5 * Eigen::MatrixXd::Identity(3,3) * dt;
    V.block<3, 3>(6, 0) =  0.5 * delta_q_i.toRotationMatrix() * dt;
    V.block<3, 3>(6, 3) =  0.5 * -delta_q_j.toRotationMatrix() * R_a_j_x  * dt * 0.5 * dt;
    V.block<3, 3>(6, 6) =  0.5 * delta_q_j.toRotationMatrix() * dt;
    V.block<3, 3>(6, 9) =  V.block<3, 3>(6, 3);
    V.block<3, 3>(9, 12) = Eigen::MatrixXd::Identity(3,3) * dt;
    V.block<3, 3>(12, 15) = Eigen::MatrixXd::Identity(3,3) * dt;

    jacobian_ = F * jacobian_;
    covariance_ = F * covariance_ * F.transpose() + V * noise_ * V.transpose();
  }

  delta_p_i = delta_p_j;
  delta_v_i = delta_v_j;
  delta_q_i = delta_q_j;
  delta_q_i.normalize();
}

void IntegrationBase::propagate(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
{
  midPointIntegration(dt, acc_prv_, gyr_prv_, acc, gyr, 
                      linearized_ba_, linearized_bg_,
                      delta_p_, delta_q_, delta_v_, 1);
  
  // dataCheck(acc_prv_, gyr_prv_, acc, gyr);

  sum_dt_ += dt;
  acc_prv_ = acc;
  gyr_prv_ = gyr;   
}

void IntegrationBase::dataCheck(
    const Eigen::Vector3d &acc_i, const Eigen::Vector3d &gyr_i,
    const Eigen::Vector3d &acc_j, const Eigen::Vector3d &gyr_j)
{

  double acc_mul = acc_i.normalized().transpose() * acc_j.normalized(); 
  double gyr_mul = gyr_i.normalized().transpose() * gyr_j.normalized(); 

  double acc_angle = acos(acc_mul) * 180.0f / M_PI;
  double gyr_angle = acos(gyr_mul) * 180.0f / M_PI;
  if(acc_angle > sigma_acc_ && acc_angle < 90.0f)
  {
    sigma_acc_ = acc_angle;
    std::cout << "sigma: " << acc_angle << "; " << gyr_angle << std::endl;
  }
  if(gyr_angle > sigma_gyr_ && gyr_angle < 90.0f)
  {
    sigma_gyr_ = gyr_angle;
    std::cout << "sigma: " << acc_angle << "; " << gyr_angle << std::endl;
  }

}

Eigen::Matrix<double, 15, 1> IntegrationBase::evaluate(
    const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi, 
    const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
    const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj, 
    const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj)
{
  Eigen::Matrix<double, 15, 1> residuals;

  Eigen::Matrix3d dp_dba = jacobian_.block<3, 3>(O_P, O_BA);
  Eigen::Matrix3d dp_dbg = jacobian_.block<3, 3>(O_P, O_BG);

  Eigen::Matrix3d dq_dbg = jacobian_.block<3, 3>(O_R, O_BG);

  Eigen::Matrix3d dv_dba = jacobian_.block<3, 3>(O_V, O_BA);
  Eigen::Matrix3d dv_dbg = jacobian_.block<3, 3>(O_V, O_BG);

  Eigen::Vector3d dba = Bai - linearized_ba_;
  Eigen::Vector3d dbg = Bgi - linearized_bg_;

  Eigen::Quaterniond corrected_delta_q = delta_q_ * Utility::deltaQ(dq_dbg * dbg);
  Eigen::Vector3d corrected_delta_v = delta_v_ + dv_dba * dba + dv_dbg * dbg;
  Eigen::Vector3d corrected_delta_p = delta_p_ + dp_dba * dba + dp_dbg * dbg;

  residuals.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * G * sum_dt_ * sum_dt_ + Pj - Pi - Vi * sum_dt_) - corrected_delta_p;
  residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
  residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (G * sum_dt_ + Vj - Vi) - corrected_delta_v;
  residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
  residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;
  return residuals;
}
