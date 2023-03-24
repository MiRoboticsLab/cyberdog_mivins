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

#include <deque>
#include <iostream>
#include <mivins/common/types.h>

namespace mivins
{

    class OdomCalibration
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<OdomCalibration> Ptr;

        Eigen::Vector3d vel_scale;
        Eigen::Vector3d gyr_scale;
        
        /// Camera-ODOM delay: delay_imu_cam = cam_ts - cam_ts_delay
        double delay_odom = 0.0f;

        /// Maximum delay camera-odom
        double max_odom_delta_t = 0.01f;

        double linear_velocity_n = 0.001f;

        double angular_velocity_n = 0.001f;

        Transformation T_B_O;

        double odom_rate = 20;

        OdomCalibration() = default;
        ~OdomCalibration() = default;

        inline void print(const std::string &s = "Odom Calibration: ") const
        {
            std::cout << s << std::endl
                      << "  delay_odom = " << delay_odom << std::endl
                      << "  vel_scale = " << vel_scale.transpose() << std::endl
                      << "  gyr_scale = " << gyr_scale.transpose() << std::endl
                      << "  max_odom_delta_t = " << max_odom_delta_t << std::endl
                      << "  linear_velocity_n = " << linear_velocity_n << std::endl
                      << "  angular_velocity_n = " << angular_velocity_n << std::endl
                      << "  T_imu_odom = \n" << T_B_O << std::endl;
        }
    };

    struct OdomMeasurement
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double timestamp_ = 0.0f; ///< In seconds.
        Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d linear_velocity_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d angular_velocity_ = Eigen::Vector3d::Zero();
        OdomMeasurement() {}
        OdomMeasurement(
            const double timestamp,
            const Eigen::Vector3d &linear_velocity,
            const Eigen::Vector3d &angular_velocity)
            : timestamp_(timestamp), linear_velocity_(linear_velocity), angular_velocity_(angular_velocity)
        {
        }
        
        OdomMeasurement(
            const double timestamp,
            const Eigen::Quaterniond& orientation,
            const Eigen::Vector3d& position,
            const Eigen::Vector3d& linear_velocity,
            const Eigen::Vector3d& angular_velocity)
            : timestamp_(timestamp)
            , orientation_(orientation)
            , position_(position)
            , linear_velocity_(linear_velocity)
            , angular_velocity_(angular_velocity)
        {
        }
    };
    typedef std::deque<OdomMeasurement,
                       Eigen::aligned_allocator<OdomMeasurement>>
        OdomMeasurements;

} // namespace mivins
