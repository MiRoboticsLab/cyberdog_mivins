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

#pragma once

#include <map>
#include <queue>
#include <deque>
#include <mutex>
#include <memory>
#include <string>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "interpolation.h"
#include "data_align.h"

namespace mivins
{

struct LooseCoupleOptions
{
  int kf_vio_size = 50;

  float init_time_thresh = 3.0f;

  int ab_continuous_cnt = 3;

  float ab_backtrack_ts = 3.0f;

  float ab_delta_yaw = 10.0f;

  float ab_dist_ratio = 1.5f;

  float ab_direct_angle = 45.0f;

  bool use_imu_rot = false;

  LooseCoupleOptions(){};

  void print()
  {
    std::cout << "###############################################\n";
    std::cout << "LooseCoupleOptions: \n";
    std::cout << "kf_vio_size: " << kf_vio_size << "\n";
    std::cout << "init_time_thresh: " << init_time_thresh << "\n";
    std::cout << "ab_continuous_cnt: " << ab_continuous_cnt << "\n";
    std::cout << "ab_backtrack_ts: " << ab_backtrack_ts << "\n";
    std::cout << "ab_delta_yaw: " << ab_delta_yaw << "\n";
    std::cout << "ab_dist_ratio: " << ab_dist_ratio << "\n";
    std::cout << "ab_direct_angle: " << ab_direct_angle << "\n";
    std::cout << "use_imu_rot: " << use_imu_rot << "\n";
  }
};


class LooseCouple
{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  typedef std::shared_ptr<LooseCouple> Ptr;

  LooseCouple(const LooseCoupleOptions &options, 
              const Eigen::Matrix4d &Tbo);
  ~LooseCouple();
  
  void setAlignHandler(const DataAlign::Ptr &align_ptr);

  void setGyrBias(const Eigen::Vector3d &gyr_bias);

  void addFrameBundle(const double timestamp, 
                  const Eigen::Matrix4d &vio_pos, 
                  const bool is_keyframe, 
                  const bool init_state,
                  const bool is_backend_ok = true);

  Eigen::Matrix4d getFusedPose();

  Eigen::Matrix4d getFusedPose(const double img_ts);

  bool getOdomData(const double timstamp, Eigen::Matrix4d &odom_pose);

  void initWithOdom(const bool init_with_odom);

  bool isAbnormal();

  void reset();

  bool first_init_ = true;
  bool use_vio_pose_ = true;

private:

  bool abnormalDetection();

  Eigen::Matrix4d abnormalProcess(const double timestamp);

  Eigen::Matrix4d getMatchedPose(int i, bool imu_pose = true);

  Eigen::Matrix4d getMatchedPose(double ts, bool imu_pose = true);

  Eigen::Matrix4d getFusedPoseWithOdom(const double timestamp);

  Eigen::Matrix3d getDeltaRotationWithImu(
      const loose_couple::ImuDatas &imu_datas);

  Eigen::Matrix4d getDeltaPoseWithOdom(
      const loose_couple::OdomData &odom_data1, 
      const loose_couple::OdomData &odom_data2, 
      const bool imu_pose = true);

  Eigen::Matrix4d getDeltaPoseWithOdom(bool imu_pose = true);

  void getDeltaPoseWithScale(const loose_couple::OdomData &odom_data1, 
          const loose_couple::OdomData &odom_data2, Eigen::Matrix4d &delta_pose);

  inline Eigen::Matrix4d inverse(Eigen::Matrix4d pose)
  {
    Eigen::Matrix4d pose_inv = Eigen::Matrix4d::Identity();
    pose_inv.block<3, 3>(0, 0) = pose.block<3, 3>(0, 0).transpose();
    pose_inv.block<3, 1>(0, 3) = -pose.block<3, 3>(0, 0).transpose() * pose.block<3, 1>(0, 3);
    return pose_inv;
  }

  LooseCoupleOptions options_;

  loose_couple::VioData prv_vio_;
  loose_couple::VioData cur_vio_;

  int ab_cnt_ = 0;
  bool is_abnormal_ = false;

  double first_init_ts_ = 0.0f;

  bool prv_state_ = false;
  bool use_imu_rot_ = false;
  bool init_with_odom_ = true;

  std::deque<loose_couple::VioData> kf_vio_data_;

  std::vector<loose_couple::VioData> vio_data_;

  std::shared_ptr<loose_couple::Interpolation> interpolation_ptr_;
  std::shared_ptr<DataAlign> align_ptr_;

  Eigen::Matrix4d Tbo_, Tob_; 

  Eigen::Matrix4d global_pose_;
  std::deque<loose_couple::VioData> global_data_;

  Eigen::Vector3d gyr_bias_;

  FILE *fp_log_;
};

}
