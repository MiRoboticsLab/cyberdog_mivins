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

#include "initial/initial_alignment.h"

InitialAlignment::InitialAlignment()
{

}

InitialAlignment::~InitialAlignment()
{

}

void InitialAlignment::solveGyroscopeBias(
    std::map<double, ImageFrame> &all_image_frame, 
    std::vector<StateGroup> &states)
{
  Eigen::Matrix3d A;
  Eigen::Vector3d b;
  Eigen::Vector3d delta_bg;
  A.setZero();
  b.setZero();
  std::map<double, ImageFrame>::iterator frame_i;
  std::map<double, ImageFrame>::iterator frame_j;
  for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++)
  {
    frame_j = next(frame_i);
    Eigen::MatrixXd tmp_A(3, 3);
    tmp_A.setZero();
    Eigen::VectorXd tmp_b(3);
    tmp_b.setZero();
    Eigen::Quaterniond q_ij(frame_i->second.R.transpose() * frame_j->second.R);
    tmp_A = frame_j->second.pre_integration->jacobian_.template block<3, 3>(O_R, O_BG);
    tmp_b = 2 * (frame_j->second.pre_integration->delta_q_.inverse() * q_ij).vec();
    A += tmp_A.transpose() * tmp_A;
    b += tmp_A.transpose() * tmp_b;
  }
  delta_bg = A.ldlt().solve(b);
  std::cout << "gyroscope bias initial calibration " << delta_bg.transpose() << "\n";

  for (int i = 0; i <= WINDOW_SIZE; i++)
    states[i].Bg += delta_bg;

  for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end( ); frame_i++)
  {
    frame_j = next(frame_i);
    frame_j->second.pre_integration->repropagate(Eigen::Vector3d::Zero(), states[0].Bg);
  }
}

void InitialAlignment::solveGyroscopeBias(
  std::map<double, ImageFrame> &all_image_frame, std::vector<StateGroup> &states,
  const Eigen::Matrix3d &rio, const Eigen::Vector3d &tio)
{
  Eigen::Quaterniond q_io(rio);
  Eigen::Quaterniond q_oi = q_io.inverse();

  Eigen::Matrix3d A;
  Eigen::Vector3d b;
  Eigen::Vector3d delta_bg;
  A.setZero();
  b.setZero();
  std::map<double, ImageFrame>::iterator frame_i;
  std::map<double, ImageFrame>::iterator frame_j;
 
  for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++)
  {
    frame_j = next(frame_i);
    Eigen::MatrixXd tmp_A(3, 3);
    tmp_A.setZero();
    Eigen::VectorXd tmp_b(3);
    tmp_b.setZero();
    Eigen::Quaterniond delta_q = frame_j->second.odom_integration->delta_q_;
    Eigen::Quaterniond q_ij(frame_i->second.R.transpose() * frame_j->second.R);
    q_ij = q_oi * q_ij * q_io; 

    tmp_A = frame_j->second.odom_integration->jacobian_.template block<3, 3>(3, 6);
    tmp_b = 2 * (delta_q.inverse() * q_ij).vec();
    A += tmp_A.transpose() * tmp_A;
    b += tmp_A.transpose() * tmp_b;
  }
  delta_bg = A.ldlt().solve(b);
  std::cout << "gyroscope bias initial calibration " << delta_bg.transpose() << "\n";

  for (int i = 0; i <= WINDOW_SIZE; i++)
  {
    states[i].odom_Bg += delta_bg;
    std::cout << i << ": " << states[i].odom_Bg.transpose() << std::endl;     
  }

  for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end( ); frame_i++)
  {
    frame_j = next(frame_i);
    frame_j->second.odom_integration->repropagate(states[0].odom_Bg);
  }
}

Eigen::MatrixXd InitialAlignment::tangentBasis(Eigen::Vector3d &g0)
{
  Eigen::Vector3d b, c;
  Eigen::Vector3d a = g0.normalized();
  Eigen::Vector3d tmp(0, 0, 1);
  if(a == tmp)
    tmp << 1, 0, 0;
  b = (tmp - a * (a.transpose() * tmp)).normalized();
  c = a.cross(b);
  Eigen::MatrixXd bc(3, 2);
  bc.block<3, 1>(0, 0) = b;
  bc.block<3, 1>(0, 1) = c;
  return bc;
}

void InitialAlignment::refineGravity(std::map<double, ImageFrame> &all_image_frame, Eigen::Vector3d &g, Eigen::VectorXd &x)
{
  Eigen::Vector3d g0 = g.normalized() * G.norm();
  Eigen::Vector3d lx, ly;

  int all_frame_count = all_image_frame.size();
  int n_state = all_frame_count * 3 + 2 + 1;

  Eigen::MatrixXd A{n_state, n_state};
  A.setZero();
  Eigen::VectorXd b{n_state};
  b.setZero();

  std::map<double, ImageFrame>::iterator frame_i;
  std::map<double, ImageFrame>::iterator frame_j;

  for(int k = 0; k < 4; k++)
  {
    Eigen::MatrixXd lxly(3, 2);
    lxly = tangentBasis(g0);
    int i = 0;
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
    {
      frame_j = next(frame_i);

      Eigen::MatrixXd tmp_A(6, 9);
      tmp_A.setZero();
      Eigen::VectorXd tmp_b(6);
      tmp_b.setZero();

      double dt = frame_j->second.pre_integration->sum_dt_;


      tmp_A.block<3, 3>(0, 0) = -dt * Eigen::Matrix3d::Identity();
      tmp_A.block<3, 2>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Eigen::Matrix3d::Identity() * lxly;
      tmp_A.block<3, 1>(0, 8) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100.0;     
      tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p_ + frame_i->second.R.transpose() * frame_j->second.R * TIC[0] - TIC[0] - frame_i->second.R.transpose() * dt * dt / 2 * g0;

      tmp_A.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
      tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
      tmp_A.block<3, 2>(3, 6) = frame_i->second.R.transpose() * dt * Eigen::Matrix3d::Identity() * lxly;
      tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v_ - frame_i->second.R.transpose() * dt * Eigen::Matrix3d::Identity() * g0;

      Eigen::Matrix<double, 6, 6> cov_inv = Eigen::Matrix<double, 6, 6>::Zero();
      //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
      //MatrixXd cov_inv = cov.inverse();
      cov_inv.setIdentity();

      Eigen::MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
      Eigen::VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

      A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
      b.segment<6>(i * 3) += r_b.head<6>();

      A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>();
      b.tail<3>() += r_b.tail<3>();

      A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
      A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
    }

    A = A * 1000.0;
    b = b * 1000.0;
    x = A.ldlt().solve(b);
    Eigen::VectorXd dg = x.segment<2>(n_state - 3);
    g0 = (g0 + lxly * dg).normalized() * G.norm();
    //double s = x(n_state - 1);
  }
  g = g0;
}

void InitialAlignment::refineGravityWithOdom(std::map<double, ImageFrame> &all_image_frame, Eigen::Vector3d &g, Eigen::VectorXd &x)
{
  Eigen::Vector3d g0 = g.normalized() * G.norm();
  Eigen::Vector3d lx, ly;
  int all_frame_count = all_image_frame.size();
  int n_state = all_frame_count * 3 + 2 + 1;

  Eigen::MatrixXd A{n_state, n_state};
  Eigen::VectorXd b{n_state};
  A.setZero();
  b.setZero();

  std::map<double, ImageFrame>::iterator frame_i;
  std::map<double, ImageFrame>::iterator frame_j;
  for(int k = 0; k < 4; k++)
  {
    Eigen::MatrixXd lxly(3, 2);
    lxly = tangentBasis(g0);
    int i = 0;
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
    {
      frame_j = next(frame_i);

      Eigen::MatrixXd tmp_A(9, 9);
      Eigen::VectorXd tmp_b(9);
      tmp_A.setZero();
      tmp_b.setZero();

      double dt = frame_j->second.pre_integration->sum_dt_;

      tmp_A.block<3, 3>(0, 0) = -dt * Eigen::Matrix3d::Identity();
      tmp_A.block<3, 2>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Eigen::Matrix3d::Identity() * lxly;
      tmp_A.block<3, 1>(0, 8) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100.0;
      tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p_ 
                    + frame_i->second.R.transpose() * frame_j->second.R * TIC[0] 
                    - TIC[0] - frame_i->second.R.transpose() * dt * dt / 2 * g0;

      tmp_A.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
      tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
      tmp_A.block<3, 2>(3, 6) = frame_i->second.R.transpose() * dt * Eigen::Matrix3d::Identity() * lxly;
      tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v_ 
                    - frame_i->second.R.transpose() * dt * Eigen::Matrix3d::Identity() * g0;

      tmp_A.block<3,1>(6, 8) = (frame_i->second.R * RIO).transpose() * (frame_j->second.T - frame_i->second.T) / 100;
      tmp_b.block<3, 1>(6, 0) = frame_j->second.odom_integration->delta_p_ - RIO.transpose() * (TIC[0] - TIO)
                    + (frame_i->second.R * RIO).transpose() * frame_j->second.R * (TIC[0] - TIO);

      Eigen::Matrix<double, 9, 9> cov_inv = Eigen::Matrix<double, 9, 9>::Zero();
      //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
      //MatrixXd cov_inv = cov.inverse();
      cov_inv.setIdentity();

      Eigen::MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
      Eigen::VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

      A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
      b.segment<6>(i * 3) += r_b.head<6>();

      A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>();
      b.tail<3>() += r_b.tail<3>();

      A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
      A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
    }

    A = A * 1000.0;
    b = b * 1000.0;
    x = A.ldlt().solve(b);
    Eigen::VectorXd dg = x.segment<2>(n_state - 3);
    g0 = (g0 + lxly * dg).normalized() * G.norm();
    //double s = x(n_state - 1);
  }
  g = g0;
}

bool InitialAlignment::linearAlignment(std::map<double, ImageFrame> &all_image_frame, Eigen::Vector3d &g, Eigen::VectorXd &x)
{
  int all_frame_count = all_image_frame.size();
  int n_state = all_frame_count * 3 + 3 + 1;

  Eigen::MatrixXd A{n_state, n_state};
  A.setZero();
  Eigen::VectorXd b{n_state};
  b.setZero();

  std::map<double, ImageFrame>::iterator frame_i;
  std::map<double, ImageFrame>::iterator frame_j;

  int i = 0;
  for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
  {
    frame_j = next(frame_i);

    Eigen::MatrixXd tmp_A(6, 10);
    tmp_A.setZero();
    Eigen::VectorXd tmp_b(6);
    tmp_b.setZero();

    double dt = frame_j->second.pre_integration->sum_dt_;

    tmp_A.block<3, 3>(0, 0) = -dt * Eigen::Matrix3d::Identity();
    tmp_A.block<3, 3>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Eigen::Matrix3d::Identity();
    tmp_A.block<3, 1>(0, 9) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100.0;     
    tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p_ + frame_i->second.R.transpose() * frame_j->second.R * TIC[0] - TIC[0];
    //cout << "delta_p   " << frame_j->second.pre_integration->delta_p_.transpose() << endl;
    tmp_A.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
    tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
    tmp_A.block<3, 3>(3, 6) = frame_i->second.R.transpose() * dt * Eigen::Matrix3d::Identity();
    tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v_;
    //cout << "delta_v   " << frame_j->second.pre_integration->delta_v_.transpose() << endl;

    Eigen::Matrix<double, 6, 6> cov_inv = Eigen::Matrix<double, 6, 6>::Zero();
    //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
    //MatrixXd cov_inv = cov.inverse();
    cov_inv.setIdentity();

    Eigen::MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
    Eigen::VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

    A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
    b.segment<6>(i * 3) += r_b.head<6>();

    A.bottomRightCorner<4, 4>() += r_A.bottomRightCorner<4, 4>();
    b.tail<4>() += r_b.tail<4>();

    A.block<6, 4>(i * 3, n_state - 4) += r_A.topRightCorner<6, 4>();
    A.block<4, 6>(n_state - 4, i * 3) += r_A.bottomLeftCorner<4, 6>();
  }

  A = A * 1000.0;
  b = b * 1000.0;
  x = A.ldlt().solve(b);
  double s = x(n_state - 1) / 100.0;
  printf("estimated scale: %f\n", s);
  g = x.segment<3>(n_state - 4);
  std::cout << "result g     " << g.norm() << "; " << g.transpose() << std::endl;
  std::cout << "desired g     " << G.norm() << "; " << G.transpose() << std::endl;
  if(fabs(g.norm() - G.norm()) > 0.5 || s < 0)
  {
    return false;
  }

  refineGravity(all_image_frame, g, x);
  s = (x.tail<1>())(0) / 100.0;
  (x.tail<1>())(0) = s;
  std::cout << "refine     " << g.norm() << " " << g.transpose() << std::endl;
  return s > 0.0f;
}

bool InitialAlignment::linearAlignmentWithOdom(std::map<double, ImageFrame> &all_image_frame, Eigen::Vector3d &g, Eigen::VectorXd &x)
{
  int all_frame_count = all_image_frame.size();
  int n_state = all_frame_count * 3 + 3 + 1;

  Eigen::MatrixXd A{n_state, n_state};
  Eigen::VectorXd b{n_state};
  A.setZero();
  b.setZero();

  std::map<double, ImageFrame>::iterator frame_i;
  std::map<double, ImageFrame>::iterator frame_j;

  int i = 0;
  for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
  {
    frame_j = next(frame_i);

    Eigen::MatrixXd tmp_A(9, 10);
    Eigen::VectorXd tmp_b(9);
    tmp_A.setZero();
    tmp_b.setZero();

    double dt = frame_j->second.pre_integration->sum_dt_;

    tmp_A.block<3, 3>(0, 0) = -dt * Eigen::Matrix3d::Identity();
    tmp_A.block<3, 3>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Eigen::Matrix3d::Identity();
    tmp_A.block<3, 1>(0, 9) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100.0;     
    tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p_ 
                  + frame_i->second.R.transpose() * frame_j->second.R * TIC[0] - TIC[0];
    //cout << "delta_p   " << frame_j->second.pre_integration->delta_p_.transpose() << endl;
    
    tmp_A.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
    tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
    tmp_A.block<3, 3>(3, 6) = frame_i->second.R.transpose() * dt * Eigen::Matrix3d::Identity();
    tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v_;
    //cout << "delta_v   " << frame_j->second.pre_integration->delta_v_.transpose() << endl;

    tmp_A.block<3, 1>(6, 9) = (frame_i->second.R * RIO).transpose() * (frame_j->second.T - frame_i->second.T) / 100;
    tmp_b.block<3, 1>(6, 0) = frame_j->second.odom_integration->delta_p_  - RIO.transpose() * (TIC[0] - TIO)
                  + (frame_i->second.R * RIO).transpose() * frame_j->second.R * (TIC[0] - TIO);

    Eigen::Matrix<double, 9, 9> cov_inv = Eigen::Matrix<double, 9, 9>::Zero();
    //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
    //MatrixXd cov_inv = cov.inverse();
    cov_inv.setIdentity();

    Eigen::MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
    Eigen::VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

    A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
    b.segment<6>(i * 3) += r_b.head<6>();

    A.bottomRightCorner<4, 4>() += r_A.bottomRightCorner<4, 4>();
    b.tail<4>() += r_b.tail<4>();

    A.block<6, 4>(i * 3, n_state - 4) += r_A.topRightCorner<6, 4>();
    A.block<4, 6>(n_state - 4, i * 3) += r_A.bottomLeftCorner<4, 6>();
  }

  A = A * 1000.0;
  b = b * 1000.0;
  x = A.ldlt().solve(b);
  double s = x(n_state - 1) / 100.0;
  printf("estimated scale: %f\n", s);
  g = x.segment<3>(n_state - 4);
  std::cout << "result g     " << g.norm() << "; " << g.transpose() << std::endl;
  std::cout << "desired g     " << G.norm() << "; " << G.transpose() << std::endl;
  if(fabs(g.norm() - G.norm()) > 0.5 || s < 0)
  {
    return false;
  }

  refineGravityWithOdom(all_image_frame, g, x);
  s = (x.tail<1>())(0) / 100.0;
  (x.tail<1>())(0) = s;
  std::cout << "refine     " << g.norm() << " " << g.transpose() << std::endl;
  if(s < 0.0 )
    return false;
  else
    return true;
}

bool InitialAlignment::linearAlignmentWithOdom(std::map<double, ImageFrame> &all_image_frame, Eigen::VectorXd &x)
{
  double A = 0.0f;
  double b = 0.0f;

  std::map<double, ImageFrame>::iterator frame_i;
  std::map<double, ImageFrame>::iterator frame_j;

  int i = 0;
  for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
  {
    frame_j = next(frame_i);

    Eigen::MatrixXd tmp_A(3, 1);
    Eigen::VectorXd tmp_b(3);
    tmp_A.setZero();
    tmp_b.setZero();

    tmp_A = (frame_i->second.R * RIO).transpose() * (frame_j->second.T - frame_i->second.T) / 100;
    tmp_b = frame_j->second.odom_integration->delta_p_  + RIO.transpose() * (TIO - TIC[0])
        + (frame_i->second.R * RIO).transpose() * frame_j->second.R *(TIC[0] - TIO);

    Eigen::MatrixXd r_A = tmp_A.transpose() *  tmp_A;
    Eigen::VectorXd r_b = tmp_A.transpose() *  tmp_b;

    A += r_A(0, 0);
    b +=r_b(0);
  }

  double s = b / A / 100.0f;
  x = Eigen::VectorXd(1);
  (x.tail<1>())(0) = s;    
  return s > 0.0f;
}

bool InitialAlignment::visualAlignment(std::map<double, ImageFrame> &all_image_frame, 
            std::vector<StateGroup> &states, Eigen::Vector3d &g, 
            Eigen::VectorXd &x, bool use_imu, bool use_odom)
{
  if(use_imu)
    solveGyroscopeBias(all_image_frame, states);

  if(use_imu && !use_odom && linearAlignment(all_image_frame, g, x))
    return true;
  else if(use_imu && use_odom && linearAlignmentWithOdom(all_image_frame, g, x))
    return true;
  else if(!use_imu && use_odom && linearAlignmentWithOdom(all_image_frame, x))
    return true;

  return false;
}
