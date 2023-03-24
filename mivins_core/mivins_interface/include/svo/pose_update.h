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
#include <queue>
#include <deque>
#include <vector>
#include <mutex>
#include <memory>
#include <string>
#include <fstream>  
#include <iostream>    

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <kindr/minimal/quat_transformation.h>

using Transformation = kindr::minimal::QuatTransformation;
using Quaternion = kindr::minimal::RotationQuaternion;
using PoseDeque = std::deque<Transformation>;

namespace mivins
{
class TransformationOptimizer;

enum class PoseUpdateStage
{
    kStart,
    kInitializing,
    kAdjusting
};

struct PoseUpdateOptions
{
    int ini_pose_num = 1000;
    int adj_pose_num = 500;
    int opt_pose_num = 100;
    int adj_z_n = 30;

    double opt_outlier_th = 3.0;
    double opt_inlier_error = 1.0;
    double opt_inlier_ratio = 0.6;
    double t_prior_lambda = 0.5;

    double moving_th = 0.1;
    double initia_th = 1.0;

    double adj_t1  = 1.0;
    double adj_t2  = 0.5;
    double adj_r1  = M_PI * 1.0/3.0;
    double adj_r2  = M_PI * 2.0/3.0;

    bool adjust_pose  = true;
    bool compensate_z = true;

    PoseUpdateOptions(){};
};


class PoseUpdate
{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  typedef std::shared_ptr<PoseUpdate> Ptr;

  PoseUpdate(const PoseUpdateOptions &options);

  ~PoseUpdate();

  void updatePose(Transformation &Twb_i, Transformation &Twb_o, bool is_kf);

  PoseUpdateStage getStage();

  Eigen::Matrix3d getRot();
  Eigen::Vector3d getTrl();
  Eigen::Vector3d getCompZ();

  Transformation getTransformation();
  Transformation getLastTarPose();

private:
    void preProcessData();
    void setUpdateStage();

    bool checkInitiaCond();
    bool checkAdjustCond();
    bool poseEnough();

    bool isMoving();

    void computeTransformation();
    void computeZDCompensation();
    
    PoseDeque vio_pose_;
    PoseDeque tar_pose_;
    Eigen::Matrix3Xd vio_pt_;
    Eigen::Matrix3Xd tar_pt_;

    Eigen::Vector3d traveled_dis_;
    double traveled_ang_;

    Eigen::Matrix3d R_;
    Eigen::Vector3d t_;
    Eigen::Vector3d delta_z_;
    
    PoseUpdateOptions options_;
    PoseUpdateStage stage_;
    std::shared_ptr<TransformationOptimizer> transformation_optimizer_;

    std::ofstream trace_pose_update_;
    bool begin_init_ = false;
    bool flag = false;
};

}
