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
#include <mivins/common/odom_calibration.h>
#include <mivins/camera_models/yaml-serialization.h>

namespace mivins
{

    struct OdomHandlerOptions
    {
        bool temporal_stationary_check = false;
        double temporal_window_length_sec_ = 0.5;
    };

    enum class OdomTemporalStatus
    {
        kStationary,
        kMoving,
        kUnkown
    };

    extern const std::map<OdomTemporalStatus, std::string> Odom_temporal_status_names_;

    class ChannelOdom
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<ChannelOdom> Ptr;
        typedef std::mutex mutex_t;
        typedef std::unique_lock<mutex_t> ulock_t;

        ChannelOdom();

        ChannelOdom(const OdomCalibration &odom_calib);

        ~ChannelOdom();

        OdomMeasurements GetMeasurementsCopy() const
        {
            ulock_t lock(measurements_mut_);
            return measurements_;
        }

        bool GetMeasurements(
            const double old_cam_timestamp, // seconds
            const double new_cam_timestamp, // seconds
            const bool delete_old_measurements,
            OdomMeasurements &measurements);

        bool GetMeasurementsContainingEdges(const double frame_timestamp, // seoncds
                                            OdomMeasurements &measurements,
                                            const bool remove_measurements);

        bool GetClosestMeasurement(
            const double timestamp,
            OdomMeasurement &measurement) const;

        bool AddOdomMeasurement(const OdomMeasurement &measurement);
        bool AddOdomMeasurementProcessed(const OdomMeasurement &measurement);

        void reset();

        double GetLatestTimestamp() const
        {
            ulock_t lock(measurements_mut_);
            return measurements_.front().timestamp_;
        }

        bool WaitTill(const double timestamp_sec, const double timeout_sec = 1.0);

        static OdomCalibration LoadCalibrationFromFile(const std::string &filename);

        //for following
        bool GetLatestPose(Transformation &T_w_o);
        bool GetCorresPose(const double frame_timestamp, Transformation &T_w_o);
        bool GetLatestPoseProcessed(Transformation &T_w_o);
        bool GetCorresPoseProcessed(const double frame_timestamp, Transformation &T_w_o);
        bool GetLatestOrienation(const double frame_timestamp, Eigen::Quaterniond &ori);
        OdomMeasurement InterplateOdom(OdomMeasurement &odom1, OdomMeasurement &odom2, const double timestamp);
        OdomMeasurement InterplateOdomOuter(OdomMeasurement &odom1, OdomMeasurement &odom2, const double timestamp);
        Eigen::Quaterniond QuatSlerp(Eigen::Quaterniond qa, Eigen::Quaterniond qb, double scalar);

    public:
        OdomCalibration odom_calib_;

    private:
        mutable mutex_t measurements_mut_;
        OdomMeasurements measurements_; ///< Newest measurement is at the front of the list
        OdomMeasurements measurements_processed_;
        OdomMeasurements temporal_odom_window_;

    };

} // namespace mivins
