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

#include <memory> // std::shared_ptr
#include <mutex>  // std::mutex
#include <iostream>
#include <fstream>
#include <deque>
#include <map>
#include <mivins/common/types.h>
#include <mivins/common/transformation.h>
#include <mivins/common/imu_calibration.h>

namespace mivins
{

    class PreintegratedImuMeasurement
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d omega_bias_;
        Eigen::Vector3d acc_bias_;
        Eigen::Vector3d delta_t_ij_;
        Eigen::Vector3d delta_v_ij_;
        Quaternion delta_R_ij_;
        double dt_sum_;

        PreintegratedImuMeasurement(
            const Eigen::Vector3d &omega_bias,
            const Eigen::Vector3d &acc_bias);

        /// Add single measurements to be integrated
        void AddMeasurement(const ImuMeasurement &m);

        /// Add many measurements to be integrated
        void AddMeasurements(const ImuMeasurements &ms);

    private:
        bool last_imu_measurement_set_;
        ImuMeasurement last_imu_measurement;
    };

    struct IMUHandlerOptions
    {
        bool temporal_stationary_check = false;
        double temporal_window_length_sec_ = 0.5;
        double stationary_acc_sigma_thresh_ = 10e-4;
        double stationary_gyr_sigma_thresh_ = 6e-5;
    };

    enum class IMUTemporalStatus
    {
        kStationary,
        kMoving,
        kUnkown
    };

    extern const std::map<IMUTemporalStatus, std::string> imu_temporal_status_names_;

    class ChannelImu
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<ChannelImu> Ptr;
        typedef std::mutex mutex_t;
        typedef std::unique_lock<mutex_t> ulock_t;

        IMUHandlerOptions options_;
        ImuCalibration imu_calib_;
        ImuInitialization imu_init_;

        // TODO: make private
        mutable mutex_t bias_mut_;
        Eigen::Vector3d acc_bias_;   //!< Accleration bias used during preintegration
        Eigen::Vector3d omega_bias_; //!< Angular rate bias values used during preintegration

        ChannelImu(
            const ImuCalibration &imu_calib,
            const ImuInitialization &imu_init,
            const IMUHandlerOptions &options);
        ~ChannelImu();

        const Eigen::Vector3d &GetAccelerometerBias() const
        {
            ulock_t lock(bias_mut_);
            return acc_bias_;
        }

        const Eigen::Vector3d &GetGyroscopeBias() const
        {
            ulock_t lock(bias_mut_);
            return omega_bias_;
        }

        void SetAccelerometerBias(const Eigen::Vector3d &acc_bias)
        {
            ulock_t lock(bias_mut_);
            acc_bias_ = acc_bias;
        }

        void SetGyroscopeBias(const Eigen::Vector3d &omega_bias)
        {
            ulock_t lock(bias_mut_);
            omega_bias_ = omega_bias;
        }

        ImuMeasurements GetMeasurementsCopy() const
        {
            ulock_t lock(measurements_mut_);
            return measurements_;
        }

        /// Get IMU measurements in some time interval. Note that you have to provide
        /// the camera timestamps. Internally, given the calibration it corrects the
        /// timestamps for delays.
        bool GetMeasurements(
            const double old_cam_timestamp, // seconds
            const double new_cam_timestamp, // seconds
            const bool delete_old_measurements,
            ImuMeasurements &measurements);

        /// Get IMU measurements up to  for the exact borders.
        /// Note that you have to provide the camera timestamp.
        /// Internally, given the calibration it corrects the timestamps for delays.
        /// The returned measurement will cover the full timeinterval
        /// (GetMeasurements only gives newest measurement smaller than new_cam_timestamp
        bool GetMeasurementsContainingEdges(const double frame_timestamp, // seoncds
                                            ImuMeasurements &measurements,
                                            const bool remove_measurements);

        bool GetClosestMeasurement(
            const double timestamp,
            ImuMeasurement &measurement) const;

        // deprecated, use preintegrated imu measurement!
        /// Gets relative transformation in IMU coordinate frame
        bool GetRelativeRotationPrior(
            const double old_cam_timestamp,
            const double new_cam_timestamp,
            bool delete_old_measurements,
            bool use_euler_integral,
            Quaternion &R_oldimu_newimu);

        bool GetAngularVelocity(
            double timestamp,
            Eigen::Vector3d &omega) const;

        /// Assumes we are in hover condition and estimates the inital orientation by
        /// estimating the gravity direction. The yaw direction is not deterministic.
        bool GetInitialAttitude(
            double timestamp,
            Quaternion &R_imu_world) const;

        bool GetInitialAttitude(
            Quaternion& R_imu_world) const;
            
        bool getInitialAttitude(
            Quaternion& R_imu_world) const;

        bool AddImuMeasurement(const ImuMeasurement &measurement);

        bool LoadImuMeasurementsFromFile(const std::string &filename);

        static Eigen::Matrix3d IntegrateGyroMeasurement(
            const Eigen::Vector3d &omega_measured,
            const Eigen::Matrix3d &R_cam_imu,
            const double delta_t);

        static ImuCalibration LoadCalibrationFromFile(const std::string &filename);
        static ImuInitialization LoadInitializationFromFile(const std::string &filename);

        void reset();

        double GetLatestTimestamp() const
        {
            ulock_t lock(measurements_mut_);
            return measurements_.front().timestamp_;
        }

        bool WaitTill(const double timestamp_sec, const double timeout_sec = 1.0);

        IMUTemporalStatus CheckTemporalStatus(const double time_sec);

    private:
        mutable mutex_t measurements_mut_;
        ImuMeasurements measurements_; ///< Newest measurement is at the front of the list
        ImuMeasurements temporal_imu_window_;
        std::ofstream ofs_; //!< File stream for tracing the received measurments
    };

} // namespace mivins
