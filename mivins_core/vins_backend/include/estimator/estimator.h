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
 
#include <mutex>
#include <queue>
#include <thread>

#include <ceres/ceres.h>
#include <unordered_map>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "estimator/frame_manager.h"
#include "estimator/motion_detector.h"

#include "common/parameters.h"
#include "common/common_lib.h"
#include "common/color_print.h"

#include "initial/initializer.h"

#include "factor/imu_factor.h"
#include "factor/odom_factor.h"
// #include "factor/odom_factor_s1d.h"
// #include "factor/odom_factor_s3d.h"
#include "factor/marginalization_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/diff_bundle_diff_cam_factor.h"
#include "factor/diff_bundle_same_cam_factor.h"
#include "factor/same_bundle_diff_cam_factor.h"

#include <condition_variable>

using namespace mivins;

struct ImuData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  double timestamp = 0.0f;
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();

  ImuData() {}
  
  ImuData(const double _timestamp,
      const Eigen::Vector3d& _linear_acceleration,
      const Eigen::Vector3d& _angular_velocity)
  : timestamp(_timestamp)
  , linear_acceleration(_linear_acceleration)
  , angular_velocity(_angular_velocity)
  {}
};

struct OdomData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  double timestamp = 0.0f;
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();

  OdomData() {}
  
  OdomData(const double _timestamp,
      const Eigen::Vector3d& _linear_velocity,
      const Eigen::Vector3d& _angular_velocity)
  : timestamp(_timestamp)
  , linear_velocity(_linear_velocity)
  , angular_velocity(_angular_velocity)
  {}

  OdomData(const double _timestamp,
      const Eigen::Quaterniond& _orientation,
      const Eigen::Vector3d& _position,
      const Eigen::Vector3d& _linear_velocity,
      const Eigen::Vector3d& _angular_velocity)
  : timestamp(_timestamp)
  , orientation(_orientation)
  , position(_position)
  , linear_velocity(_linear_velocity)
  , angular_velocity(_angular_velocity)
  {}
};

class Estimator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<Estimator> Ptr;

  Estimator(const CameraBundlePtr& camera_bundle);

  Estimator(const CameraBundlePtr& camera_bundle, const VinsBackendOptions& backend_options);
  
  ~Estimator();

  void reset();
  
  void getMargedKF(FrameBundlePtr &marged_frames);

  void setImuHandler(const std::shared_ptr<ImuHandler> imu_handler);

  void setOdomHandler(const std::shared_ptr<OdomHandler> odom_handler);

  void inputOdom(const OdomData &odom_data);

  void inputOdom(const double ts,
                 const Eigen::Quaterniond &odom_rot,
                 const Eigen::Vector3d &odom_pos,
                 const Eigen::Vector3d &odom_vel, 
                 const Eigen::Vector3d &odom_gyr);

  void inputImu(const ImuData &imu_data);

  void inputImu(const double ts, 
                const Eigen::Vector3d &linear_acceleration, 
                const Eigen::Vector3d &angular_velocity);

  void addFrameBundle(const FrameBundlePtr& new_frames);

  void addInitFrameBundle(const FrameBundlePtr &new_frames);

  void optimizePose(const FrameBundlePtr &new_frames);

  void optimizeStructure(const FrameBundlePtr &new_frames, int max_n_pts, int max_iter);

  void createMotionDetector(const VinsMotionDetectorOptions& motion_detector_options);
  
  bool motionDetect(const FrameBundlePtr& last_frames, 
                    const FrameBundlePtr& new_frames,
                    const ImuMeasurements& imu_measurements,
                    const OdomMeasurements& odom_measurements);

  void valueInterpolation(
      const double ts1, const Eigen::Vector3d &val1, 
      const double ts2, const Eigen::Vector3d &val2,
      const double ts, Eigen::Vector3d &val);

  void poseInterpolation(
      const double ts1, const Eigen::Quaterniond &q1, const Eigen::Vector3d &p1, 
      const double ts2, const Eigen::Quaterniond &q2, const Eigen::Vector3d &p2,
      const double ts, Eigen::Quaterniond &q, Eigen::Vector3d &p);

  const Eigen::Vector3d getOdomVelScale();

  const Eigen::Vector3d getOdomGyrScale();

  const double getTdOdom();

  const double getTdImu();

  const bool isBackendOK();

  bool use_imu_ = false;
  bool use_odom_ = false;

  VinsBackendOptions options_;

  std::vector<StateGroup> states_;
  StateGroup tmp_state_;

  Matrix3dVec ric_;
  Vector3dVec tic_;

  Eigen::Matrix3d rio_, roi_;
  Eigen::Vector3d tio_, toi_;

  SolverFlag solver_flag_;

  double frame_ts_;

  FrameBundlePtr init_ref_frames_;

  std::mutex state_mutex_;
  
  Eigen::Vector3d g_;

  Eigen::Matrix3d rot_org_, rot_opt_;
  Eigen::Vector3d trans_org_, trans_opt_;

  FrameManagerPtr frame_manager_;
  CameraBundlePtr camera_bundle_;

  std::map<double, ImageFrame> all_image_frame_;    

private:

  void clearState();

  void setParameter();

  void createDoubleArray();

  void deleteDoubleArray();

  void process();
  
  void stateCheck();
  
  void initWindowState();

  void updateLandmarks();

  void updateFramePose();

  void updateFrameBundles();

  void updateMonoInitFrameBundles();

  void updateInitFrameBundles(const FrameBundlePtr &new_frames);

  void addImageFrame(const FrameBundlePtr &new_frames);

  void setMargedKF(const FrameBundlePtr &marged_frames);

  void processFrameBundle(const FrameBundlePtr& new_frames);

  void monoInitialize(const FrameBundlePtr &new_frames);

  void stereoInitialize(const FrameBundlePtr &new_frames);

  void tripleWithDepthInitialize(const FrameBundlePtr &new_frames);

  void tripleWithDepthInitializeWithSfm(const FrameBundlePtr &new_frames);

  void tripleWithStereoInitialize(const FrameBundlePtr &new_frames);

  void initFirstImuPose(std::vector<ImuData> &imu_vec);

  void processImuAndOdom(const FrameBundlePtr &new_frames);

  void processOdom(const double prv_ts, const double cur_ts, 
                  std::vector<OdomData> &odom_vec,
                  Eigen::Matrix3d &R, Eigen::Vector3d &P);

  void processImu(const double prv_ts, const double cur_ts, std::vector<ImuData> imu_vec, 
                  Eigen::Matrix3d &R, Eigen::Vector3d &P, Eigen::Vector3d &V,
                  Eigen::Vector3d &Ba, Eigen::Vector3d &Bg);

  void interpolationImu(const int i, const double cur_ts, std::vector<ImuData> &imu_vec);

  void interpolationOdom(const int i, const double cur_ts, std::vector<OdomData> &odom_vec);

  void addVisualResidualForOptimize(ceres::Problem &problem, ceres::LossFunction *loss_function);

  void addVisualResidualForOptimizeFull(ceres::Problem &problem, ceres::LossFunction *loss_function);

  void addVisualResidualForOptimizePart(ceres::Problem &problem, ceres::LossFunction *loss_function);

  void addVisualResidualForMarginalize(MarginalizationInfo *marginalization_info, ceres::LossFunction *loss_function);
  
  void addVisualResidualForMarginalizeFull(MarginalizationInfo *marginalization_info, ceres::LossFunction *loss_function);

  void addVisualResidualForMarginalizePart(MarginalizationInfo *marginalization_info, ceres::LossFunction *loss_function);

  bool imuAvailable(const double ts);

  bool odomAvailable(const double ts);

  bool getImuMeasurements(const double t0, const double t1, std::vector<ImuData> &imu_vec);

  bool getOdomMeasurements(const double t0, const double t1, std::vector<OdomData> &odom_vec);

  void setOdomWeight(const IntegrationOdomPtr &integration_odom, const int tracking_cnt);

  void setImuWeight(const IntegrationBasePtr &integration_imu, const int tracking_cnt);

  Eigen::Quaterniond quatSlerp(Eigen::Quaterniond qa, Eigen::Quaterniond qb, double scalar);

  void margInThread();

  void slideWindow();
  void slideWindowFirstNew();
  void slideWindowSecondNew();
  void slideWindowOld();
  void optimization();
  void marginalization();
  void vector2double(const bool opt);
  void double2vector();
  bool failureDetection();
  void reTriangulate();
  void initPoseByPnP();
  void resetKeyFrameState(const StateGroup &state);
  void updateKeyFrameState(const StateGroup &state);
  
  void printPoses();

  void printPose(int frame_count);

  void printPose(const FrameBundlePtr frame_bundle);

  void printExtrinsic();

  int cam_size_ = 0;
  int grain_size_ = 1;
  int frame_count_ = 0;

  double td_imu_ = 0.0f;
  double td_odom_  = 0.0f;
  double initial_ts_ = 0.0f;
  double last_marged_ts_ = 0.0f;
  double plane_d_ = 0.0f;
  double cur_ts_ = 0.0f;
  double prv_ts_ = 0.0f;

  bool first_imu_ = false;
  bool first_odom_ = false;
  bool start_thread_ = false;
  bool backend_quit_ = false;
  bool failure_occur_ = false;
  bool init_first_pose_ = false;
  bool cam_ex_estimate_ = false;
  bool odom_ex_estimate_ = false;
  bool propagate_imu_ = true;
  bool is_kf_ = false;

  std::thread process_thread_;

  FrameBundlePtr marged_frames_;

  std::mutex buf_mutex_;
  std::mutex process_mutex_;

  std::condition_variable con_;

  std::vector<int> prev_obs_count_;
  std::vector<size_t> removed_feature_;
  std::vector<size_t> opt_feature_ids_;
  std::map<size_t, size_t> para_feature_map_;

  std::queue<ImuData> imu_queue_;
  std::queue<OdomData> odom_queue_;
  std::queue<FrameBundlePtr> frame_bundles_buf_;

  double **para_Pose_ = nullptr;
  double **para_SpeedBias_ = nullptr;
  double **para_InvDepth_ = nullptr;
  double **para_CamEx_Pose_ = nullptr;
  double *para_Td_imu_ = nullptr;
  double *para_OdomEx_Pose_ = nullptr;
  double *para_plane_d_ = nullptr;

  Eigen::Vector3d acc_prv_, gyr_prv_;

  OdomData odom_data_prv_;

  Eigen::Matrix3d back_R0_, last_R_, last_R0_;
  Eigen::Vector3d back_P0_, last_P_, last_P0_;

  Eigen::Vector3d odom_vel_scale_;
  Eigen::Vector3d odom_gyr_scale_;

  // just for backups
  std::unordered_map<size_t, FeaturePerId> feature_per_ids_;

  MotionEstimator motion_estimator_;

  IntegrationBasePtr tmp_pre_integration_ = nullptr;
  IntegrationOdomPtr tmp_odom_integration_ = nullptr;

  MargFlag  marg_flag_;

  MarginalizationInfo *last_marginalization_info_ = nullptr;
  std::vector<double *> last_marginalization_parameter_blocks_;

  std::shared_ptr<Initializer> initializer_;

  std::shared_ptr<ImuHandler> imu_handler_;
  std::shared_ptr<OdomHandler> odom_handler_;
  std::shared_ptr<VinsMotionDetector> motion_detector_;

  std::shared_ptr<OptimizePose> optimize_pose_;

  std::thread marg_thread_;
  bool marg_done_ = true;

  // FILE *fp_be_ts_cost_;
  std::ofstream save_be_ts_;
};

using EstimatorPtr = std::shared_ptr<Estimator>;
