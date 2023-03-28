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

#include "common/parameters.h"
#include "common/common_lib.h"
#include "estimator/estimator.h"
#include "estimator/outlier_rejection.h"

namespace mivins
{

class VinsBackendInterface: public BundleAdjustment
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<VinsBackendInterface> Ptr;

  VinsBackendInterface(const VinsBackendOptions& vins_backend_options,
              const VinsMotionDetectorOptions& motion_detector_options,
              const CameraBundlePtr& camera_bundle, const int cam_size);

  ~VinsBackendInterface();

  void reset() override;

  bool isEstimatorValid() override;

  int getWindowsSize() override;

  FrameBundlePtr getFrameBundle(int i) override;

  void setImuHandler(const std::shared_ptr<ImuHandler> imu_handler) override;

  void setOdomHandler(const std::shared_ptr<OdomHandler> odom_handler) override;
  
  void getMotionPrior(const FrameBundlePtr& new_frames, 
          const FrameBundlePtr& last_frames, 
          bool& have_motion_prior) override;

  bool tryToInitialize(const FrameBundlePtr& frames_cur) override;

  bool inputImu(const FrameBundlePtr& new_frames) override;

  bool inputOdom(const FrameBundlePtr& new_frames) override;

  bool addFrameBundle(const FrameBundlePtr& new_frames) override;

  void updateLatestStates(const FrameBundlePtr& new_frames, 
        const FrameBundlePtr& last_frames) override;

  FrameBundlePtr getRefFrames() override;

  void getMargedKF(FrameBundlePtr& marged_KF) override;

  Eigen::Vector3d getOdomVelScale() override;

  Eigen::Vector3d getOdomGyrScale() override;

  bool isBackendOK() override;

private:

  // void createFeatureTrackers(  
  //     const CameraBundlePtr& camera_bundle,
  //     const DetectorOptions& detector_options,
  //     const FeatureTrackerOptions& tracker_options);

  void updateLandmarks(const FrameBundlePtr& frame_bundle);

  bool motionCheck(const FrameBundlePtr& last_frames, 
       const FrameBundlePtr& new_frames);

  void drawInitMonoMatches();

  void drawInitStereoMatches();

  void printInitResult();

  bool predictWithImu(
    const ImuMeasurements& imu_measurements, 
    const double ts_start, const double ts_end);

  bool predictWithImuMedian(
    const ImuMeasurements& imu_measurements, 
    const double ts_start, const double ts_end);

  bool predictWithImuRK4(
    const ImuMeasurements& imu_measurements, 
    const double ts_start, const double ts_end);

  bool predictWithOdom(
    const OdomMeasurements& odom_measurements, 
    const double ts_start, const double ts_end);

  void getMotionPriorWithImu(
    const FrameBundlePtr& new_frames, 
    const FrameBundlePtr& last_frames, 
    bool& have_motion_prior);

  void getMotionPriorWithOdom(
    const FrameBundlePtr& new_frames, 
    const FrameBundlePtr& last_frames, 
    bool& have_motion_prior);

  void alignImuAndOdom(
    ImuMeasurements &imu_measurements, 
    OdomMeasurements &odom_measurements);

  bool getImuMeasurements(const double timestamp, 
    ImuMeasurements& imu_measurements, bool erase);

  bool getOdomMeasurements(const double timestamp, 
    OdomMeasurements& odom_measurements, bool erase);

  bool processImu(const FrameBundlePtr& new_frames);

  bool motionDetect(const FrameBundlePtr& last_frames, 
        const FrameBundlePtr& new_frames,
        const ImuMeasurements& imu_measurements,
        const OdomMeasurements& odom_measurements);

  bool inputImuAndOdom(const FrameBundlePtr& new_frames);


  const Eigen::Matrix3d JacobianL(const Eigen::Vector3d& w);

  const Eigen::Matrix3d JacobianR(const Eigen::Vector3d& w);

  const Eigen::Matrix3d expMatrix(const Eigen::Vector3d & omega);

private:

  /* data */
  std::shared_ptr<ImuHandler> imu_handler_ = nullptr;
  std::shared_ptr<OdomHandler> odom_handler_ = nullptr;
  std::shared_ptr<Estimator> vins_estimator_ = nullptr;
  std::shared_ptr<VinsOutlierRejection> outlier_rejection_ = nullptr;

  FrameBundlePtr ref_frames_;

  Eigen::Vector3d g_;
  
  StateGroup latest_state_;

  double input_imu_ts_prv_ = 0.0f;
  double input_odom_ts_prv_ = 0.0f;

  bool get_motion_prior_with_imu_ = true;
  bool get_motion_prior_with_odom_ = true;
  bool get_motion_prior_with_odompose_ = false;
};

}


