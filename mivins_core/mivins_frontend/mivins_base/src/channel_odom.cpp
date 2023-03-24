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

#include "mivins/channel_odom.h"

#include <numeric>

#include <mivins/utils/math_utils.h>
#include <mivins/utils/timer.h>
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <yaml-cpp/yaml.h>
#pragma diagnostic pop

namespace mivins
{

    const std::map<OdomTemporalStatus, std::string> odom_temporal_status_names_{
        {OdomTemporalStatus::kStationary, "Stationary"},
        {OdomTemporalStatus::kMoving, "Moving"},
        {OdomTemporalStatus::kUnkown, "Unknown"}};

    ChannelOdom::ChannelOdom()
    {
    }

    ChannelOdom::ChannelOdom(const OdomCalibration &odom_calib)
    {
        odom_calib_ = odom_calib;
    }

    ChannelOdom::~ChannelOdom()
    {
    }

    bool ChannelOdom::GetMeasurementsContainingEdges(
        const double frame_timestamp, // seconds
        OdomMeasurements &extracted_measurements,
        const bool remove_measurements)
    {
        ulock_t lock(measurements_mut_);
        if (measurements_.empty())
        {
            LOG(WARNING) << "don't have any odom measurements!";
            return false;
        }

        const double t = frame_timestamp - odom_calib_.delay_odom;

        OdomMeasurements::iterator it = measurements_.begin();
        for (; it != measurements_.end(); ++it)
        {
            if (it->timestamp_ < t)
            {
                if (it == measurements_.begin())
                {
                    LOG(WARNING) << "need a newer measurement for interpolation!";
                    return false;
                }
                //decrement iterator again to point to element >= t
                --it;
                break;
            }
        }

        // copy affected measurements
        extracted_measurements.insert(extracted_measurements.begin(),
                                      it, measurements_.end());

        // check
        if (extracted_measurements.size() < 2)
        {
            LOG(WARNING) << "need older odom measurements!";
            extracted_measurements.clear();
            return false;
        }

        if (remove_measurements)
        {
            measurements_.erase(it + 2, measurements_.end());
        }

        return true;
    }

    bool ChannelOdom::GetMeasurements(
        const double old_cam_timestamp,
        const double new_cam_timestamp,
        const bool delete_old_measurements,
        OdomMeasurements &measurements)
    {
        assert(new_cam_timestamp > old_cam_timestamp);
        ulock_t lock(measurements_mut_);
        if (measurements_.empty())
        {
            LOG(WARNING) << "don't have any odom measurements!";
            return false;
        }

        // Substract camera delay to get odom timestamp.
        const double t1 = old_cam_timestamp;
        const double t2 = new_cam_timestamp;

        // Find the right measurements for integration, note that the newest
        // measurement is at the front of the list!
        OdomMeasurements::iterator it1 = measurements_.end(); // older timestamp
        OdomMeasurements::iterator it2 = measurements_.end(); // newer timestamp
        bool it2_set = false;
        for (OdomMeasurements::iterator it = measurements_.begin();
             it != measurements_.end(); ++it)
        {
            if (!it2_set && it->timestamp_ < t2)
            {
                it2 = it;
                it2_set = true;
            }

            if (it->timestamp_ <= t1)
            {
                it1 = it;
                break;
            }
        }

        // check
        if (it1 == measurements_.end())
        {
            LOG(WARNING) << "need an older measurement for t1!";
            return false;
        }

        if (it2 == measurements_.end())
        {
            LOG(WARNING) << "need an older measurement for t2!";
            return false;
        }

        if (it1 == it2)
        {
            LOG(WARNING) << "not enough odom measurements!";
            return false;
        }

        if (t2 - it2->timestamp_ > odom_calib_.max_odom_delta_t)
        {
            LOG(WARNING) << "newest odom measurement is too old for the image "
                         << t2 - it2->timestamp_;
            return false;
        }

        // copy affected measurements
        ++it1; // make sure to copy the last element!
        measurements.insert(measurements.begin(), it2, it1);

        // change timestamp of oldest measurement
        measurements.back().timestamp_ = t1;

        // delete measurements that will not be used anymore
        if (delete_old_measurements)
        {
            measurements_.erase(it1, measurements_.end());
        }

        return true;
    }

    bool ChannelOdom::GetClosestMeasurement(
        const double timestamp,
        OdomMeasurement &measurement) const
    {
        ulock_t lock(measurements_mut_);
        if (measurements_.empty())
        {
            LOG(WARNING) << "ChannelOdom: don't have any odom measurements!";
            return false;
        }

        double dt_best = std::numeric_limits<double>::max();
        double img_timestamp_corrected = timestamp;
        for (const OdomMeasurement &m : measurements_)
        {
            const double dt = std::abs(m.timestamp_ - img_timestamp_corrected);
            if (dt < dt_best)
            {
                dt_best = dt;
                measurement = m;
            }
        }

        if (dt_best > odom_calib_.max_odom_delta_t)
        {
            LOG(WARNING) << "ChannelOdom: GetClosestMeasurement: no measurement found!"
                            " closest measurement: "
                         << dt_best * 1000.0 << "ms.";
            return false;
        }
        return true;
    }

    bool ChannelOdom::AddOdomMeasurement(
        const OdomMeasurement &m)
    {
        ulock_t lock(measurements_mut_);
        measurements_.push_front(m); // new measurement is at the front of the list!
        // if (options_.temporal_stationary_check)
        // {
        //   temporal_Odom_window_.push_front(m);
        // }

        //if there are too many odom measurements, delete the oldest
        if(measurements_.size() > 10000)
            measurements_.pop_back();
        return true;
    }

    bool ChannelOdom::AddOdomMeasurementProcessed(
        const OdomMeasurement &m)
    {
        ulock_t lock(measurements_mut_);
        measurements_processed_.push_front(m); // new measurement is at the front of the list!

        //if there are too many odom measurements, delete the oldest
        if(measurements_processed_.size() > 10000)
            measurements_processed_.pop_back();
        return true;
    }

    void ChannelOdom::reset()
    {
        ulock_t lock(measurements_mut_);
        measurements_.clear();
	measurements_processed_.clear();
        temporal_odom_window_.clear();
    }

    bool ChannelOdom::WaitTill(const double img_timestamp_sec,
                               const double timeout_sec)
    {
        vk::Timer wait_time;
        wait_time.start();
        while (this->GetLatestTimestamp() < img_timestamp_sec)
        {
            if (wait_time.stop() > timeout_sec)
            {
                LOG(ERROR) << "Did not get odom measurements";
                return false;
            }
            //wait_time.resume();
            // wait
            VLOG(50) << "Waiting for odom measurements.";
        }

        return true;
    }

    OdomCalibration ChannelOdom::LoadCalibrationFromFile(const std::string &filename)
    {
        YAML::Node data = YAML::LoadFile(filename);
        OdomCalibration calib;
        if(data["odometry_params"].IsDefined())
        {
            calib.delay_odom = data["odometry_params"]["delay_odom"].as<double>();
            calib.max_odom_delta_t = data["odometry_params"]["max_odom_delta_t"].as<double>();
            calib.linear_velocity_n = data["odometry_params"]["linear_velocity_n"].as<double>();
            calib.angular_velocity_n = data["odometry_params"]["angular_velocity_n"].as<double>();

            calib.vel_scale = Eigen::Vector3d(
                  data["odometry_params"]["vel_scale"][0].as<double>(),
                  data["odometry_params"]["vel_scale"][1].as<double>(),
                  data["odometry_params"]["vel_scale"][2].as<double>());
              
            calib.gyr_scale = Eigen::Vector3d(
                  data["odometry_params"]["gyr_scale"][0].as<double>(),
                  data["odometry_params"]["gyr_scale"][1].as<double>(),
                  data["odometry_params"]["gyr_scale"][2].as<double>());

            Eigen::Matrix4d T_B_O_raw;
            if (!YAML::safeGet(data["odometry_params"], "T_B_O", &T_B_O_raw)) 
            {
                LOG(ERROR) << "Unable to get extrinsic transformation T_B_O for odometry\n";
            }

            Eigen::Quaterniond q_B_O = Eigen::Quaterniond(T_B_O_raw.block<3, 3>(0, 0));
            q_B_O.normalize();
            
            calib.T_B_O = Transformation(q_B_O, T_B_O_raw.block<3, 1>(0, 3));
        }
        else
        {
            LOG(FATAL) << "Could not load odom calibration from file";
        }
        return calib;
    }

    //for following
    bool ChannelOdom::GetLatestPose(Transformation &T_w_o)
    {
        if(measurements_.empty())
            return false;
        Eigen::Vector3d t_w_o = measurements_.front().position_;
        Eigen::Quaterniond q_w_o = measurements_.front().orientation_;
        q_w_o.normalize();
        T_w_o = Transformation(q_w_o, t_w_o);
        return true;
    }

    //for following
    bool ChannelOdom::GetLatestPoseProcessed(Transformation &T_w_o)
    {
        if(measurements_processed_.empty())
            return false;
        Eigen::Vector3d t_w_o = measurements_processed_.front().position_;
        Eigen::Quaterniond q_w_o = measurements_processed_.front().orientation_;
        q_w_o.normalize();
        T_w_o = Transformation(q_w_o, t_w_o);
        return true;
    }

    bool ChannelOdom::GetCorresPose(const double frame_timestamp, Transformation &T_w_o)
    {
        OdomMeasurement corres_odom;

        if(measurements_.size() < 2)
            return false;

        double interval = measurements_[0].timestamp_ - measurements_[1].timestamp_;

        if(measurements_.back().timestamp_ > frame_timestamp || measurements_.front().timestamp_ < frame_timestamp - interval)
            return false;

        if(frame_timestamp > measurements_.front().timestamp_)
            corres_odom = InterplateOdomOuter(measurements_[1], measurements_[0], frame_timestamp);
        else
        {
            int idx = 0;
            for(; idx<measurements_.size(); ++idx)
            {
                if(frame_timestamp >= measurements_[idx].timestamp_)
                    break;
            }

            if(idx == 0)
                corres_odom = measurements_.front();
            else
                corres_odom = InterplateOdom(measurements_[idx], measurements_[idx-1], frame_timestamp);
        }

        if(corres_odom.position_.hasNaN() || corres_odom.orientation_.coeffs().hasNaN())
            return false;
        
        Eigen::Vector3d t_w_o = corres_odom.position_;
        Eigen::Quaterniond q_w_o = corres_odom.orientation_;
        q_w_o.normalize();
        T_w_o = Transformation(q_w_o, t_w_o);

        return true;
    }

    bool ChannelOdom::GetCorresPoseProcessed(const double frame_timestamp, Transformation &T_w_o)
    {
        OdomMeasurement corres_odom;

        if(measurements_processed_.size() < 2)
            return false;

        double interval = measurements_processed_[0].timestamp_ - measurements_processed_[1].timestamp_;

        if(measurements_processed_.back().timestamp_ > frame_timestamp || measurements_processed_.front().timestamp_ < frame_timestamp - interval)
            return false;

        if(frame_timestamp > measurements_processed_.front().timestamp_)
            corres_odom = InterplateOdomOuter(measurements_processed_[1], measurements_processed_[0], frame_timestamp);
        else
        {
            int idx = 0;
            for(; idx<measurements_processed_.size(); ++idx)
            {
                if(frame_timestamp >= measurements_processed_[idx].timestamp_)
                    break;
            }

            if(idx == 0)
                corres_odom = measurements_processed_.front();
            else
                corres_odom = InterplateOdom(measurements_processed_[idx], measurements_processed_[idx-1], frame_timestamp);
        }

        if(corres_odom.position_.hasNaN() || corres_odom.orientation_.coeffs().hasNaN())
            return false;
        
        Eigen::Vector3d t_w_o = corres_odom.position_;
        Eigen::Quaterniond q_w_o = corres_odom.orientation_;
        q_w_o.normalize();
        T_w_o = Transformation(q_w_o, t_w_o);

        return true;
    }

    bool ChannelOdom::GetLatestOrienation(const double frame_timestamp, Eigen::Quaterniond &ori)
    {
        if(measurements_.empty())
            return false;

        if(frame_timestamp >= measurements_.front().timestamp_)
        {
            ori = measurements_.front().orientation_;
            return true;
        }
        if(frame_timestamp <= measurements_.back().timestamp_)
        {
            ori = measurements_.back().orientation_;
            return true;
        }

        int idx = 0;
        for(; idx<measurements_.size(); ++idx)
        {
            if(frame_timestamp >= measurements_[idx].timestamp_)
                break;
        }
        ori = QuatSlerp(measurements_[idx].orientation_, measurements_[idx-1].orientation_, frame_timestamp);
        return true;
    }

    OdomMeasurement ChannelOdom::InterplateOdom(OdomMeasurement &odom1, OdomMeasurement &odom2, const double t)
    {
        OdomMeasurement new_odom;

        double t_range = odom2.timestamp_ - odom1.timestamp_;
        double t_delta = t - odom1.timestamp_;
        double ratio = t_delta / t_range;

        Eigen::Vector3d vel_range = odom2.linear_velocity_ - odom1.linear_velocity_;
        Eigen::Vector3d ang_range = odom2.angular_velocity_ - odom1.angular_velocity_;
        Eigen::Vector3d pos_range = odom2.position_ - odom1.position_;

        new_odom.timestamp_ = t;
        new_odom.linear_velocity_ = odom1.linear_velocity_ + ratio*vel_range;
        new_odom.angular_velocity_ = odom1.angular_velocity_ + ratio*ang_range;
        new_odom.position_ = odom1.position_ + ratio*pos_range;
        new_odom.orientation_ = QuatSlerp(odom1.orientation_, odom2.orientation_, ratio);

        return new_odom;
    }

    OdomMeasurement ChannelOdom::InterplateOdomOuter(OdomMeasurement &odom1, OdomMeasurement &odom2, const double t)
    {
        OdomMeasurement new_odom;

        double t_range = odom2.timestamp_ - odom1.timestamp_;
        double t_delta = t - odom2.timestamp_;
        double ratio = t_delta / t_range;

        Eigen::Vector3d vel_range = odom2.linear_velocity_ - odom1.linear_velocity_;
        Eigen::Vector3d ang_range = odom2.angular_velocity_ - odom1.angular_velocity_;
        Eigen::Vector3d pos_range = odom2.position_ - odom1.position_;

        new_odom.timestamp_ = t;
        new_odom.linear_velocity_ = odom2.linear_velocity_ + ratio*vel_range;
        new_odom.angular_velocity_ = odom2.angular_velocity_ + ratio*ang_range;
        new_odom.position_ = odom2.position_ + ratio*pos_range;
        Eigen::Quaterniond rot_inner = QuatSlerp(odom1.orientation_, odom2.orientation_, ratio);
        Eigen::Quaterniond rot_delta = odom1.orientation_.inverse() * rot_inner;
        rot_delta.normalize();
        new_odom.orientation_ = odom2.orientation_ * rot_delta;
   
        return new_odom;
    }

    Eigen::Quaterniond ChannelOdom::QuatSlerp(Eigen::Quaterniond qa, Eigen::Quaterniond qb, double scalar)
    {
         // quaternion to return
        Eigen::Quaterniond qm;
        // Calculate angle between them.
        double cosHalfTheta = qa.w() * qb.w() + qa.x() * qb.x() + qa.y() * qb.y() + qa.z() * qb.z();

        // If the dot product is negative, the quaternions have opposite handed-ness and slerp won't take
        // the shorter path. Fix by reversing one quaternion.
        if(cosHalfTheta < 0.0f)
        {
            qb.w() = -qb.w();
            qb.x() = -qb.x();
            qb.y() = -qb.y();
            qb.z() = -qb.z();
            cosHalfTheta = -cosHalfTheta;
        }

        // if qa=qb or qa=-qb then theta = 0 and we can return qa
        if (abs(cosHalfTheta) >= 1.0)
        {
            qm.w() = qa.w();
            qm.x() = qa.x();
            qm.y() = qa.y();
            qm.z() = qa.z();
            return qm;
        }
        // Calculate temporary values.
        double halfTheta = acos(cosHalfTheta);
        double sinHalfTheta = sqrt(1.0 - cosHalfTheta*cosHalfTheta);
        // if theta = 180 degrees then result is not fully defined
        // we could rotate around any axis normal to qa or qb
        if (fabs(sinHalfTheta) < 0.001)
        { // fabs is floating point absolute
            qm.w() = (qa.w() * 0.5 + qb.w() * 0.5);
            qm.x() = (qa.x() * 0.5 + qb.x() * 0.5);
            qm.y() = (qa.y() * 0.5 + qb.y() * 0.5);
            qm.z() = (qa.z() * 0.5 + qb.z() * 0.5);
            return qm;
        }
        
        double ratioA = sin((1 - scalar) * halfTheta) / sinHalfTheta;
        double ratioB = sin(scalar * halfTheta) / sinHalfTheta; 
        //calculate Quaternion.
        qm.w() = (qa.w() * ratioA + qb.w() * ratioB);
        qm.x() = (qa.x() * ratioA + qb.x() * ratioB);
        qm.y() = (qa.y() * ratioA + qb.y() * ratioB);
        qm.z() = (qa.z() * ratioA + qb.z() * ratioB);
        return qm;
    }

} // namespace mivins
