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

#pragma once

#include <map>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "factor/integration_base.h"
#include "factor/integration_odom.h"
#include "common/common_lib.h"

class ImageFrame
{
public:
  ImageFrame(){};
  ImageFrame(const FrameBundlePtr &_frame_bundle)
  {
    is_key_frame = false;
    frame_bundle = _frame_bundle;
  };
  
  void setPose(Eigen::Matrix3d &rot, Eigen::Vector3d &trans)
  {
    R = rot;
    T = trans;

    // Eigen::Quaterniond Q(R); 
    // Transformation T_w_i(Q, T);
    // frame_bundles_->set_T_W_B(T_w_i);
  }

  bool is_key_frame;

  Eigen::Matrix3d R;
  Eigen::Vector3d T;

  IntegrationBasePtr pre_integration;
  IntegrationOdomPtr odom_integration;
  FrameBundlePtr frame_bundle;
};

class InitialAlignment
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<InitialAlignment> Ptr;

  InitialAlignment();

  ~InitialAlignment();

  Eigen::MatrixXd tangentBasis(Eigen::Vector3d &g0);

  void refineGravity(std::map<double, ImageFrame> &all_image_frame, 
              Eigen::Vector3d &g, Eigen::VectorXd &x);

  void refineGravityWithOdom(std::map<double, ImageFrame> &all_image_frame, 
              Eigen::Vector3d &g, Eigen::VectorXd &x);

  bool linearAlignment(std::map<double, ImageFrame> &all_image_frame, 
              Eigen::Vector3d &g, Eigen::VectorXd &x);

  bool linearAlignmentWithOdom(std::map<double, ImageFrame> &all_image_frame, 
              Eigen::Vector3d &g, Eigen::VectorXd &x);

  bool linearAlignmentWithOdom(std::map<double, ImageFrame> &all_image_frame, Eigen::VectorXd &x);

  bool visualAlignment(std::map<double, ImageFrame> &all_image_frame,
              std::vector<StateGroup> &states, Eigen::Vector3d &g, 
              Eigen::VectorXd &x, bool use_imu, bool use_odom);


  void solveGyroscopeBias(std::map<double, ImageFrame> &all_image_frame, std::vector<StateGroup> &states);

  void solveGyroscopeBias(std::map<double, ImageFrame> &all_image_frame, std::vector<StateGroup> &states, 
                          const Eigen::Matrix3d &rio, const Eigen::Vector3d &tio);
};