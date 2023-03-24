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

// svo
#include <mivins/mivins_global_types.h>
#include <mivins/channel_imu.h>
#include <mivins/channel_odom.h>
#include <mivins/common/transformation.h>

namespace mivins
{

/// EXPERIMENTAL Defines interface for various bundle adjustment methods
class BundleAdjustment
{
public:
  typedef std::shared_ptr<BundleAdjustment> Ptr;

  /// Default constructor.
  BundleAdjustment(){}

  virtual ~BundleAdjustment() {}

  // no copy
  BundleAdjustment(const BundleAdjustment&) = delete;
  BundleAdjustment& operator=(const BundleAdjustment&) = delete;

  virtual void reset() = 0;

  //virtual bool needNewKF() = 0;

  virtual int getWindowsSize() = 0;

  virtual bool isEstimatorValid() = 0;

  virtual mivins::FrameBundlePtr getRefFrames() = 0;
  
  virtual mivins::FrameBundlePtr getFrameBundle(int i) = 0;

  virtual void setImuHandler(const std::shared_ptr<mivins::ChannelImu> imu_handler) = 0;

  virtual void setOdomHandler(const std::shared_ptr<mivins::ChannelOdom> odom_handler) = 0;

  virtual void getMotionPrior(const mivins::FrameBundlePtr& new_frames, 
                              const mivins::FrameBundlePtr& last_frames, 
                              bool &have_motion_prior) = 0;

  //virtual void updateState(const mivins::FrameBundlePtr& new_frames) = 0;

  virtual bool tryToInitialize(const mivins::FrameBundlePtr& frames_cur) = 0;

  virtual bool inputImu(const mivins::FrameBundlePtr& new_frames) = 0;

  virtual bool inputOdom(const mivins::FrameBundlePtr& new_frames) = 0;

  //virtual void addForLWO(const mivins::FrameBundlePtr& new_frames) = 0;
  
  virtual bool addFrameBundle(const mivins::FrameBundlePtr& new_frames) = 0;

  virtual void updateLatestStates(const mivins::FrameBundlePtr &new_frames, 
                        const mivins::FrameBundlePtr &last_frames) = 0;

  virtual void getMargedKF(mivins::FrameBundlePtr &marged_KF) = 0;
  
  virtual Eigen::Vector3d getOdomVelScale() = 0;

  virtual Eigen::Vector3d getOdomGyrScale() = 0;
  
  virtual bool isBackendOK() = 0;
};

}  // namespace svo
