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
#include <cmath>
#include <time.h>
#include <chrono>
#include <thread>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <stdlib.h>

#include <cassert>
#include <cstring>

#include <Eigen/Eigen>

#include <glog/logging.h>

// #include <svo/common/frame.h>
// #include <svo/common/types.h>
#include "global.h"

class IntegrationBase;
class IntegrationOdom;

using IntegrationBasePtr = std::shared_ptr<IntegrationBase>; 
using IntegrationOdomPtr = std::shared_ptr<IntegrationOdom>; 

using Matrix3dVec = std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>;
using Vector3dVec = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;
using QuaterniondVec = std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>;

template <typename T, size_t M, size_t N>
static T (&makeSubArray(T (&orig)[M], size_t o))[N]
{
  return (T (&)[N])(*(orig + o));
}

// static const double getTimestamp(const FrameBundlePtr& frame_bundle)
// {
//   assert(frame_bundle != nullptr);
//   return frame_bundle->getMinTimestampSeconds();
// }

// static Eigen::Matrix3d getImu2WorldRot(const FrameBundlePtr& frame_bundle)
// {
//   return frame_bundle->get_T_W_B().getRotationMatrix();
// }

// static Eigen::Vector3d getImu2WorldPos(const FrameBundlePtr& frame_bundle)
// {
//   return frame_bundle->get_T_W_B().getPosition();
// }

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

struct StateGroup
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  double td_imu;
  double td_odom;
  Eigen::Vector3d P;
  Eigen::Vector3d V;
  Eigen::Matrix3d R;
  Eigen::Vector3d Ba;
  Eigen::Vector3d Bg;
  Eigen::Vector3d odom_Bg;
  FrameBundlePtr         frame_bundle;
  IntegrationBasePtr     pre_integration;
  IntegrationOdomPtr     odom_integration;

  std::vector<double> imu_dt_buf;
  Vector3dVec         imu_acc_buf;
  Vector3dVec         imu_gyr_buf;
  
  std::vector<double> odom_dt_buf;
  Vector3dVec         odom_vel_buf;
  Vector3dVec         odom_gyr_buf;
  Vector3dVec         odom_pos_buf;
  QuaterniondVec      odom_rot_buf;

  int tracked_feat_cnt = 0;

  StateGroup()
  {
    reset();
  }

  ~StateGroup()
  {
    reset();
  }

  void reset()
  {
    td_imu = 0.0f;
    td_odom = 0.0f;

    tracked_feat_cnt = 0;

    R.setIdentity();
    P.setZero();
    V.setZero();
    Ba.setZero();
    Bg.setZero();
    odom_Bg.setZero();

    imu_dt_buf.clear();
    imu_acc_buf.clear();
    imu_gyr_buf.clear();

    odom_dt_buf.clear();
    odom_vel_buf.clear();
    odom_gyr_buf.clear();
    odom_pos_buf.clear();
    odom_rot_buf.clear();

    frame_bundle = nullptr;
    pre_integration = nullptr;
    odom_integration = nullptr;
  }

  /*
  void setTimestamp()
  {
    assert(this->frame_bundle != nullptr);
    this->timestamp_sec = this->frame_bundle->getMinTimestampSeconds();
  }
  */

};

class Timer
{
public:

  typedef std::map<std::string, std::chrono::time_point< std::chrono::system_clock >>           StringTimepointMap;
  typedef std::map<std::string, std::chrono::time_point< std::chrono::system_clock >>::iterator StringTimepointMapIter;

  Timer()
  {
    (findTimepoint( std::string( " " )))->second = timerNow();
  }

  void tic( std::string str = std::string( " " ) )
  {
    findTimepoint(str.append(getThreadIdStr()))->second = timerNow();
  }

  double toc(std::string str = std::string( " " ), int retick = 1 )
  {
    StringTimepointMapIter it = findTimepoint(str.append(getThreadIdStr())) ;

    std::chrono::duration<double> time_diff = timerNow() - it->second;
    if (retick)
    {
      it->second = timerNow();
    }
    return time_diff.count() * 1000;
  }

  std::string toc2string( std::string str = std::string( " " ), int retick = 1 )
  {
    sprintf(temp_char_, "[Timer](%s): %s cost time = %.2f ms ",  
      getThreadIdStr().c_str(), str.c_str(), toc(str, retick));

    return std::string(temp_char_);
  }

  void print(std::string str = std::string( " " ), int retick = 1)
  {
    std::cout << toc2string(str, retick) << "\n";
  }

private:

  std::chrono::time_point< std::chrono::system_clock > timerNow()
  {
    return std::chrono::system_clock::now();
  }

  StringTimepointMapIter findTimepoint(const std::string &str)
  {
    StringTimepointMapIter it = map_str_timepoint_.find( str );
    if ( it == map_str_timepoint_.end() )
    {
      map_str_timepoint_.insert( std::make_pair( str, timerNow() ) );
      return map_str_timepoint_.find( str );
    }
    else
    {
      return it;
    }
  }

  std::string getThreadIdStr()
  {
    if(with_thread_id_)
    {
      std::stringstream ss;
      ss << std::this_thread::get_id();
      return  ss.str();
    }
    else
    {
      return std::to_string(0);
    }
  }

  uint64_t getThreadId()
  {
    return std::stoull(getThreadIdStr());
  }

private:

    StringTimepointMap map_str_timepoint_;
    char temp_char_[4096];
    int  with_thread_id_ = 1;
};

class Utility
{
public:
  template <typename Derived>
  static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
  {
    typedef typename Derived::Scalar Scalar_t;

    Eigen::Quaternion<Scalar_t> dq;
    Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
    half_theta /= static_cast<Scalar_t>(2.0);
    dq.w() = static_cast<Scalar_t>(1.0);
    dq.x() = half_theta.x();
    dq.y() = half_theta.y();
    dq.z() = half_theta.z();
    return dq;
  }

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
  {
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
    ans << typename Derived::Scalar(0), -q(2), q(1),
        q(2), typename Derived::Scalar(0), -q(0),
        -q(1), q(0), typename Derived::Scalar(0);
    return ans;
  }

  template <typename Derived>
  static Eigen::Quaternion<typename Derived::Scalar> positify(const Eigen::QuaternionBase<Derived> &q)
  {
      //printf("a: %f %f %f %f", q.w(), q.x(), q.y(), q.z());
      //Eigen::Quaternion<typename Derived::Scalar> p(-q.w(), -q.x(), -q.y(), -q.z());
      //printf("b: %f %f %f %f", p.w(), p.x(), p.y(), p.z());
      //return q.template w() >= (typename Derived::Scalar)(0.0) ? q : Eigen::Quaternion<typename Derived::Scalar>(-q.w(), -q.x(), -q.y(), -q.z());
      return q;
  }

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived> &q)
  {
    Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
    Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
    ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
    ans.template block<3, 1>(1, 0) = qq.vec(), ans.template block<3, 3>(1, 1) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(qq.vec());
    return ans;
  }

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived> &p)
  {
    Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
    Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
    ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
    ans.template block<3, 1>(1, 0) = pp.vec(), ans.template block<3, 3>(1, 1) = pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - skewSymmetric(pp.vec());
    return ans;
  }

  static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
  {
    Eigen::Vector3d n = R.col(0);
    Eigen::Vector3d o = R.col(1);
    Eigen::Vector3d a = R.col(2);

    Eigen::Vector3d ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr / M_PI * 180.0;
  }

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr)
  {
    typedef typename Derived::Scalar Scalar_t;

    Scalar_t y = ypr(0) / 180.0 * M_PI;
    Scalar_t p = ypr(1) / 180.0 * M_PI;
    Scalar_t r = ypr(2) / 180.0 * M_PI;

    Eigen::Matrix<Scalar_t, 3, 3> Rz;
    Rz << cos(y), -sin(y), 0,
          sin(y), cos(y), 0,
          0, 0, 1;

    Eigen::Matrix<Scalar_t, 3, 3> Ry;
    Ry << cos(p), 0., sin(p),
          0., 1., 0.,
          -sin(p), 0., cos(p);

    Eigen::Matrix<Scalar_t, 3, 3> Rx;
    Rx << 1., 0., 0.,
          0., cos(r), -sin(r),
          0., sin(r), cos(r);

    return Rz * Ry * Rx;
  }

  static Eigen::Matrix3d g2R(const Eigen::Vector3d &g)
  {
    Eigen::Matrix3d R0;
    Eigen::Vector3d ng1 = g.normalized();
    Eigen::Vector3d ng2{0, 0, 1.0};
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
    return R0;
  }

  template <size_t N>
  struct uint_
  {
  };

  template <size_t N, typename Lambda, typename IterT>
  void unroller(const Lambda &f, const IterT &iter, uint_<N>)
  {
    unroller(f, iter, uint_<N - 1>());
    f(iter + N);
  }

  template <typename Lambda, typename IterT>
  void unroller(const Lambda &f, const IterT &iter, uint_<0>)
  {
    f(iter);
  }

  template <typename T>
  static T normalizeAngle(const T& angle_degrees) {
    T two_pi(2.0 * 180);
    if (angle_degrees > 0)
    return angle_degrees -
        two_pi * std::floor((angle_degrees + T(180)) / two_pi);
    else
      return angle_degrees +
          two_pi * std::floor((-angle_degrees + T(180)) / two_pi);
  };
};
