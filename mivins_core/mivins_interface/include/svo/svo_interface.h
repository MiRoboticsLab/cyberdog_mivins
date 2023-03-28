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

#include <thread>
#include <vector>
#include <Eigen/Core>
#include <unordered_map>
#include <opencv2/opencv.hpp>

#include <kindr/minimal/quat_transformation.h>
#include "mivins/ekf_3dof/head_tracker.h"
#include <svo/loose_couple.h>
#include <svo/pose_update.h>

using Transformation = kindr::minimal::QuatTransformation;
using Quaternion = kindr::minimal::RotationQuaternion;

namespace mivins
{

    // forward declarations
    class ChannelImu;
    class ChannelOdom;
    class LooseCouple;
    class DataAlign;
    class ChannelFrameBase;
    class VinsBackendInterface;

    enum class PipelineType
    {
        kRgbd,
        kMono,
        kStereo,
        kTripleWithStereo,
        kTripleWithDepth
    };

    enum MotionMode
    {
        POWER_ON = 0,
        LYING = 101,
        STANDING = 111,
        WALKING = 303,
        JUMPING = 126,
        UpStair = 340,
        DownStair = 341,
        Climbing = 342
    };

    struct OdomInfo
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        double timestamp = 0.0f;
        Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
        Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();

        OdomInfo() {}
        
        OdomInfo(const double _timestamp,
            const Eigen::Vector3d& _linear_velocity,
            const Eigen::Vector3d& _angular_velocity)
        : timestamp(_timestamp)
        , linear_velocity(_linear_velocity)
        , angular_velocity(_angular_velocity)
        {}

        OdomInfo(const double _timestamp,
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

        void setOrigin()
        {
            timestamp = 0.0f;
            orientation.setIdentity();
            position.setZero();
            linear_velocity.setZero();
            angular_velocity.setZero();
        }
    };

    /// SVO Interface
    class SvoInterface
    {
    public:
        PipelineType pipeline_type_;
        std::string remote_key_topic_;
        std::string remote_input_;

        // SVO modules.
        std::shared_ptr<ChannelFrameBase> svo_;
        std::shared_ptr<ChannelImu> imu_handler_;
        std::shared_ptr<ekf3dof::HeadTracker> header_tracker_;
        std::shared_ptr<ChannelOdom> odom_handler_;
        std::shared_ptr<LooseCouple> loose_couple_ = nullptr;
        std::shared_ptr<DataAlign> data_align_ = nullptr;
        std::shared_ptr<VinsBackendInterface> vins_backend_interface_;
        std::shared_ptr<PoseUpdate> pose_updater_ = nullptr;

        std::string calib_file_;
        std::string config_file_;

        // Parameters
        bool set_initial_attitude_from_gravity_ = true;

        // for following
        Transformation pose_follow_;
        Eigen::Vector3d pos_odom_last_;
        bool stable_mode_first_stage_ = false;
        bool init_stable_mode_ = false;
        int robot_status_ = -1;

        bool use_constraint_odom_ = false;
        bool use_processed_odom_ = false;
        bool odom_processed_ = false;
        OdomInfo odom_last_;
        bool update_pose_;

        // System state.
        bool quit_ = false;
        bool idle_ = false;
        bool automatic_reinitialization_ = false;

        SvoInterface(const std::string config_file,
                     const ::std::string calib_file);

        virtual ~SvoInterface();
        void savefile();
        void finishfile();

        bool setImuPrior(const int64_t timestamp_nanoseconds);
        bool setImuPrior_3dof(const int64_t timestamp_nanoseconds);
        bool setOrientationPrior(const int64_t timestamp_nanoseconds);

        int getStage();

        int getCamSize() const;

        int getPipelineType() const;

        bool imuHandlerValid() const;

        bool odomHandlerValid() const;

        int getTrackingQuality() const;

        void processImageData(const int64_t ts,
                              const std::vector<cv::Mat> &images,
                              const std::map<int, cv::Mat> &depths);

        void inputImuData(const double ts,
                          const Eigen::Vector3d &acc_imu,
                          const Eigen::Vector3d &gyr_imu);

        void inputOdomData(const double ts,
                           const Eigen::Quaterniond &orientation, 
                           const Eigen::Vector3d &position, 
                           const Eigen::Vector3d &linear_velocity,
                           const Eigen::Vector3d &angular_velocity);

        void inputStatusData(const int motion_id);
        int getRobotStatus();
                           
        bool getAlignedOdom(const double timestamp, Eigen::Matrix4d &odom_pose);
        
        bool getLatestAlignedOdom(double &timestamp, Eigen::Matrix4d &odom_pose);

        OdomInfo processOdom(const double ts,
                             const Eigen::Quaterniond &orientation, 
                             const Eigen::Vector3d &position, 
                             const Eigen::Vector3d &linear_velocity,
                             const Eigen::Vector3d &angular_velocity);

        // just for visualizer
        bool isLastKeyFrame();
        double getLastProcessingTime();
        int64_t getLastNumObservations();
        int64_t GetLastFramesTimestamp();
        //   Transformation getLastFramesImuPose();

        //   std::vector<int> getLastFramesId();
        //   std::vector<cv::Mat> getLastFramesImage();
        std::vector<std::vector<double>> GetLastFramesPose(const std::string sensor_type = "imu");
        std::vector<std::vector<double>> GetLastFramesCamPose();
        std::vector<double> GetLastFramesIMUPose();
        std::vector<double> GetCameraImuCalib();
        std::vector<double> GetImuBaselinkCalib();
        void SvoShutdown();
        //   std::vector<Eigen::MatrixXd> getLastFramesPts();

        //   std::vector<Eigen::Vector4d> getMapRegionPoints();
        //   std::unordered_map<int, Transformation> getCloseKfPoses();
        //   std::unordered_map<int, Eigen::Vector4d> getCloseKfPts();

        //   std::vector<Eigen::Vector3d> getMapSeeds();
        //   std::vector<Eigen::Vector3d> getMapSeedsUncertainty();

        //   std::vector<Transformation> getActiveKeyframePoses();

        //   std::vector<cv::Mat> getLastFramePyr(const int cam_idx);
        //   Eigen::MatrixXd getLastFrameFeaturePx(const int cam_idx);
        //   Eigen::MatrixXd getLastFrameFeatureGrad(const int cam_idx);
        //   Eigen::MatrixXi getLastFrameFeatureStatus(const int cam_idx);

        //   std::vector<cv::Mat> getNewFramePyr(const int cam_idx);
        std::map<int64_t, float> ltimes;
        std::map<int64_t, float> rtimes;
        void loadTimes(std::string file, int id_camera);
        bool exposure_time_enable;
        bool use_imu_only_for_gravityalign_;
        bool b_first_get_init_with_vio_g_ = false;
    private:    
          double prv_img_ts_ = 0.0f;
          double prv_imu_ts_ = 0.0f;
          double prv_odom_ts_ = 0.0f;
          
          bool is_abnormal_ = false;
          Transformation loose_couple_pose_;
          
          void looseCoupleProcess();

          void abnormalProcess(const int64_t timestamp_nanoseconds);

          void updatePose(const int64_t timestamp_nanoseconds);
    };

} // namespace mivins
