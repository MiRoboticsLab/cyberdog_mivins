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
#include <mutex>
#include <memory>
#include <string>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "interpolation.h"

namespace Eigen 
{

template <typename T>
using aligned_vector = std::vector<T, Eigen::aligned_allocator<T>>;

template <typename T>
using aligned_deque = std::deque<T, Eigen::aligned_allocator<T>>;

template <typename K, typename V>
using aligned_map = std::map<K, V, std::less<K>, Eigen::aligned_allocator<std::pair<K const, V>>>;

template <typename K, typename V>
using aligned_unordered_map = std::unordered_map<K, V, std::hash<K>, std::equal_to<K>,
                   Eigen::aligned_allocator<std::pair<K const, V>>>;

}  // namespace Eigen

// typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> AlignedVectorVector3d;
// typedef std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> AlignedVectorQuaterniond;

namespace loose_couple
{
struct VioData 
{
  bool is_keyframe_;
  double timestamp_;
  Eigen::Matrix4d T_w_b_;

  VioData()
  {
    timestamp_ = 0.0f;
    T_w_b_.setIdentity();
    is_keyframe_ = false;
  }

  VioData(double _timestamp, bool _is_keyframe, Eigen::Matrix4d _T_w_b)
  {
    T_w_b_ = _T_w_b;
    timestamp_ = _timestamp;
    is_keyframe_ = _is_keyframe;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct ImuData
{  
  double timestamp_;
  Eigen::Vector3d linear_acceleration_;
  Eigen::Vector3d angular_velocity_;

  ImuData() 
  {
    timestamp_ = 0.0f;
    angular_velocity_.setIdentity();
    linear_acceleration_.setIdentity();
  }
  
  ImuData(const double timestamp,
      const Eigen::Vector3d& angular_velocity,
      const Eigen::Vector3d& linear_acceleration)
  {
    timestamp_ = timestamp;
    angular_velocity_ = angular_velocity;
    linear_acceleration_ = linear_acceleration;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct OdomData
{
  double timestamp_;
  Eigen::Vector3d position_;
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d linear_velocity_;
  Eigen::Vector3d angular_velocity_;

  OdomData()
  {
    timestamp_ = 0.0f;
    position_.setZero();
    orientation_.setIdentity();
    linear_velocity_.setZero();
    angular_velocity_.setZero();
  }

  OdomData(const double _timestamp, const Eigen::Quaterniond &_orientation, 
      const Eigen::Vector3d &_position, const Eigen::Vector3d &_linear_velocity, 
      const Eigen::Vector3d &_angular_velocity)
  {
    timestamp_ = _timestamp;
    position_ = _position;
    linear_velocity_ = _linear_velocity;
    angular_velocity_ = _angular_velocity;
    orientation_ = _orientation.normalized();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::deque<ImuData, Eigen::aligned_allocator<ImuData> > ImuDatas;
}

namespace mivins
{

class DataAlign
{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  typedef std::shared_ptr<DataAlign> Ptr;

  DataAlign();

  ~DataAlign();

  void inputImu(const double timestamp,
                const Eigen::Vector3d &angular_velocity,
                const Eigen::Vector3d &linear_acceleration);

  void inputOdom(const double timestamp, const Eigen::Quaterniond &orientation,
                 const Eigen::Vector3d &position, const Eigen::Vector3d &vel, 
                 const Eigen::Vector3d &gyr);

  void align(const double img_ts);

  bool getAlignedOdom(const double timestamp, Eigen::Matrix4d &odom_pose);

  bool getLatestAlignedOdom(double &timestamp, Eigen::Matrix4d &odom_pose);

  void getImuMeasurements(const double ts0, const double ts1, loose_couple::ImuDatas &imu_data);

  std::deque<double> matched_img_imu_ts_;

  std::unordered_map<double, loose_couple::ImuDatas> matched_img_imus_;

  std::deque<double> matched_img_odom_ts_;

  std::unordered_map<double, loose_couple::OdomData> matched_img_odoms_;

private:

  void reset();

  bool imuAvailable(const double img_ts);

  bool odomAvailable(const double img_ts);

  bool imageAndImuAlign(const double img_ts);

  bool imageAndOdomAlign(const double img_ts);

  loose_couple::ImuData imuInterpolation(
    const double inter_ts,
    const loose_couple::ImuData &imu_data1, 
    const loose_couple::ImuData &imu_data2);

  void odomInterpolation(
      const loose_couple::OdomData &odom1, 
      const loose_couple::OdomData &odom2, 
      const double ts, loose_couple::OdomData &odom);

  void inputRecalOdom(const double timestamp, const Eigen::Quaterniond &orientation,
      const Eigen::Vector3d &position, const Eigen::Vector3d &linear_velocity, 
      const Eigen::Vector3d &angular_velocity);

  void recalculateOdom(
      const loose_couple::OdomData& prv_odom_org, 
      const loose_couple::OdomData& cur_odom_org, 
      const loose_couple::OdomData &prv_odom_cal, 
      loose_couple::OdomData &cur_odom_cal);

  std::mutex imu_mutex_;
  std::mutex odom_mutex_;

  loose_couple::ImuData prv_imu_;
  loose_couple::OdomData prv_odom_;

  std::deque<loose_couple::ImuData>  imu_queue_;
  std::deque<loose_couple::OdomData> odom_queue_;

  bool first_img_for_imu_ = true;
  std::deque<double> img_ts_queue_for_imu_;

  std::deque<double> img_ts_queue_for_odom_;

  unsigned long int odom_cnt_ = 0;

  std::shared_ptr<loose_couple::Interpolation> interpolation_ptr_;
  
};

}
