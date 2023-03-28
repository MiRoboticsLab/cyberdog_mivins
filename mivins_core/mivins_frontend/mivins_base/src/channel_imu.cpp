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

#include "mivins/channel_imu.h"

#include <numeric>

#include <mivins/utils/math_utils.h>
#include <mivins/utils/timer.h>
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <yaml-cpp/yaml.h>
#pragma diagnostic pop

namespace
{

    double stdVec(const std::vector<double> &v)
    {
        double sum = std::accumulate(v.begin(), v.end(), 0.0);
        double mean = sum / v.size();

        std::vector<double> diff(v.size());
        std::transform(v.begin(), v.end(), diff.begin(),
                       std::bind2nd(std::minus<double>(), mean));
        double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
        double stdev = std::sqrt(sq_sum / v.size());

        return stdev;
    }
}

namespace mivins
{

    PreintegratedImuMeasurement::PreintegratedImuMeasurement(
        const Eigen::Vector3d &omega_bias,
        const Eigen::Vector3d &acc_bias)
        : omega_bias_(omega_bias), acc_bias_(acc_bias), delta_t_ij_(Eigen::Vector3d::Zero()), delta_v_ij_(Eigen::Vector3d::Zero()), delta_R_ij_(), dt_sum_(0.0), last_imu_measurement_set_(false)
    {
    }

    void PreintegratedImuMeasurement::AddMeasurement(const ImuMeasurement &m)
    {
        if (last_imu_measurement_set_)
        {
            const double dt = m.timestamp_ - last_imu_measurement.timestamp_;
            const Eigen::Vector3d a = last_imu_measurement.linear_acceleration_ - acc_bias_;
            const Eigen::Vector3d w = last_imu_measurement.angular_velocity_ - omega_bias_;
            const Quaternion R_incr = Quaternion::Exp(w * dt);

            // second order integration:
            delta_t_ij_ += delta_v_ij_ * dt + (delta_R_ij_.Rotate(a)) * dt * dt * 0.5;
            delta_v_ij_ += delta_R_ij_.Rotate(a * dt);
            delta_R_ij_ = delta_R_ij_ * R_incr;
            dt_sum_ += dt;
        }
        last_imu_measurement_set_ = true;
        last_imu_measurement = m;
    }

    void PreintegratedImuMeasurement::AddMeasurements(const ImuMeasurements &ms)
    {
        for (const ImuMeasurement &m : ms)
        {
            AddMeasurement(m);
        }
    }

    const std::map<IMUTemporalStatus, std::string> imu_temporal_status_names_{{IMUTemporalStatus::kStationary, "Stationary"},
                                                                              {IMUTemporalStatus::kMoving, "Moving"},
                                                                              {IMUTemporalStatus::kUnkown, "Unknown"}};

    ChannelImu::ChannelImu(
        const ImuCalibration &imu_calib,
        const ImuInitialization &imu_init,
        const IMUHandlerOptions &options)
        : options_(options), imu_calib_(imu_calib), imu_init_(imu_init), acc_bias_(imu_init.acc_bias), omega_bias_(imu_init.omega_bias)
    {
    }

    ChannelImu::~ChannelImu()
    {
    }
    /// Get IMU measurements up to  for the exact borders.
    /// Note that you have to provide the camera timestamp.
    /// Internally, given the calibration it corrects the timestamps for delays.
    /// Also not that this adds all the measurements since the last call, which
    /// should correspond to the time of the last added frame
    bool ChannelImu::GetMeasurementsContainingEdges(
        const double frame_timestamp, // seconds
        ImuMeasurements &extracted_measurements,
        const bool remove_measurements)
    {
        ulock_t lock(measurements_mut_);
        if (measurements_.empty())
        {
            LOG(WARNING) << "don't have any imu measurements!";
            return false;
        }

        // Substract camera delay to get imu timestamp.
        const double t = frame_timestamp - imu_calib_.delay_imu_cam;

        // Find the first measurement newer than frame_timestamp,
        // note that the newest measurement is at the front of the list!
        ImuMeasurements::iterator it = measurements_.begin();
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
            LOG(WARNING) << "need older imu measurements!";
            extracted_measurements.clear();
            return false;
        }

        if (remove_measurements)
        {
            // delete measurements that will not be used anymore (such that we keep it+1,
            // the first frame with smaller timestamp (earlier) than frame_timestamp,
            // which will be used in interpolation in next iteration
            measurements_.erase(it + 2, measurements_.end());
        }

        return true;
    }

    bool ChannelImu::GetMeasurements(
        const double old_cam_timestamp,
        const double new_cam_timestamp,
        const bool delete_old_measurements,
        ImuMeasurements &measurements)
    {
        if(new_cam_timestamp <= old_cam_timestamp)
        {
          printf("old_cam_timestamp: %.9lf\n", old_cam_timestamp);
          printf("new_cam_timestamp: %.9lf\n", new_cam_timestamp);
          return false;
        }
        
        ulock_t lock(measurements_mut_);
        if (measurements_.empty())
        {
            LOG(WARNING) << "don't have any imu measurements!";
            return false;
        }

        // Substract camera delay to get imu timestamp.
        const double t1 = old_cam_timestamp - imu_calib_.delay_imu_cam;
        const double t2 = new_cam_timestamp - imu_calib_.delay_imu_cam;

        // Find the right measurements for integration, note that the newest
        // measurement is at the front of the list!
        ImuMeasurements::iterator it1 = measurements_.end(); // older timestamp
        ImuMeasurements::iterator it2 = measurements_.end(); // newer timestamp
        bool it2_set = false;
        for (ImuMeasurements::iterator it = measurements_.begin();
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
            LOG(WARNING) << "not enough imu measurements!";
            return false;
        }

        if (t2 - it2->timestamp_ > imu_calib_.max_imu_delta_t)
        {
            LOG(WARNING) << "newest imu measurement is too old for the image "
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

    bool ChannelImu::GetClosestMeasurement(
        const double timestamp,
        ImuMeasurement &measurement) const
    {
        ulock_t lock(measurements_mut_);
        if (measurements_.empty())
        {
            LOG(WARNING) << "ChannelImu: don't have any imu measurements!";
            return false;
        }

        double dt_best = std::numeric_limits<double>::max();
        double img_timestamp_corrected = timestamp - imu_calib_.delay_imu_cam;
        for (const ImuMeasurement &m : measurements_)
        {
            const double dt = std::abs(m.timestamp_ - img_timestamp_corrected);
            if (dt < dt_best)
            {
                dt_best = dt;
                measurement = m;
            }
        }

        if (dt_best > imu_calib_.max_imu_delta_t)
        {
            LOG(WARNING) << "ChannelImu: GetClosestMeasurement: no measurement found!"
                            " closest measurement: "
                         << dt_best * 1000.0 << "ms.";
            return false;
        }
        return true;
    }

    bool ChannelImu::GetRelativeRotationPrior(
        const double old_cam_timestamp,
        const double new_cam_timestamp,
        bool delete_old_measurements,
        bool use_euler_integral,
        Quaternion &R_oldimu_newimu)
    {
        ImuMeasurements measurements;
        if (!GetMeasurements(
                old_cam_timestamp, new_cam_timestamp, delete_old_measurements, measurements))
            return false;

        // integrate all measurements from t1 to t2
        R_oldimu_newimu.SetIdentity();
        ImuMeasurements::reverse_iterator it = measurements.rbegin();
        ImuMeasurements::reverse_iterator it_plus = measurements.rbegin();
        Eigen::Vector3d omega_prv = it->angular_velocity_;
        
        ++it_plus;
        for (; it != measurements.rend(); ++it, ++it_plus)
        {
            double dt = 0.0;
            if (it_plus == measurements.rend()) // only for newest measurement
                dt = new_cam_timestamp - imu_calib_.delay_imu_cam - it->timestamp_;
            else
                dt = it_plus->timestamp_ - it->timestamp_;

            Eigen::Vector3d omega_corrected = Eigen::Vector3d::Zero();
            if(use_euler_integral)
                omega_corrected = it->angular_velocity_ - omega_bias_;
            else
                omega_corrected = 0.5 * (it->angular_velocity_ + omega_prv) - omega_bias_;
            R_oldimu_newimu = R_oldimu_newimu * Quaternion::Exp(omega_corrected*dt);
            omega_prv = it->angular_velocity_;
        }
        return true;
    }

    bool ChannelImu::AddImuMeasurement(
        const ImuMeasurement &m)
    {
        ulock_t lock(measurements_mut_);
        measurements_.push_front(m); // new measurement is at the front of the list!
        if (options_.temporal_stationary_check)
        {
            temporal_imu_window_.push_front(m);
        }

        //if there are too many imu measurements, delete the oldest
        if(measurements_.size() > 10000)
            measurements_.pop_back();

        return true;
    }

    bool ChannelImu::LoadImuMeasurementsFromFile(const std::string &filename)
    {
        ulock_t lock(measurements_mut_);
        std::ifstream fs(filename.c_str());
        if (!fs.is_open())
        {
            LOG(WARNING) << "Could not open imu file: " << filename;
            return false;
        }

        // add all imu messages to imu-handler
        size_t n = 0;
        while (fs.good() && !fs.eof())
        {
            if (fs.peek() == '#') // skip comments
                fs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            size_t imu_id;
            double stamp, wx, wy, wz, ax, ay, az;
            fs >> imu_id >> stamp >> wx >> wy >> wz >> ax >> ay >> az;

            //    // DEBUG **************************
            //    if(imu_id % 10 != 0)
            //      continue;
            //    // ********************************

            const Eigen::Vector3d omega(wx, wy, wz);
            const Eigen::Vector3d linacc(ax, ay, az);
            const ImuMeasurement m(stamp, omega, linacc);
            measurements_.push_front(m);
            if (options_.temporal_stationary_check)
            {
                temporal_imu_window_.push_front(m);
            }
            ++n;
        }
        VLOG(2) << "ChannelImu: Loaded " << n << " measurements.";
        return true;
    }

    ImuCalibration ChannelImu::LoadCalibrationFromFile(const std::string &filename)
    {
        YAML::Node data = YAML::LoadFile(filename);
        ImuCalibration calib;
        if (data["imu_params"].IsDefined())
        {
            calib.delay_imu_cam = data["imu_params"]["delay_imu_cam"].as<double>();
            calib.max_imu_delta_t = data["imu_params"]["max_imu_delta_t"].as<double>();
            calib.saturation_accel_max = data["imu_params"]["acc_max"].as<double>();
            calib.saturation_omega_max = data["imu_params"]["omega_max"].as<double>();
            calib.gyro_noise_density = data["imu_params"]["sigma_omega_c"].as<double>();
            calib.acc_noise_density = data["imu_params"]["sigma_acc_c"].as<double>();
            calib.gyro_bias_random_walk_sigma = data["imu_params"]["sigma_omega_bias_c"].as<double>();
            calib.acc_bias_random_walk_sigma = data["imu_params"]["sigma_acc_bias_c"].as<double>();
            calib.gravity_magnitude = data["imu_params"]["g"].as<double>();
            calib.imu_rate = data["imu_params"]["imu_rate"].as<double>();
        }
        else
        {
            LOG(FATAL) << "Could not load IMU calibration from file";
        }
        return calib;
    }

    ImuInitialization ChannelImu::LoadInitializationFromFile(const std::string &filename)
    {
        YAML::Node data = YAML::LoadFile(filename);
        ImuInitialization init;
        if (data["imu_initialization"].IsDefined())
        {
            init.velocity = Eigen::Vector3d(
                data["imu_initialization"]["velocity"][0].as<double>(),
                data["imu_initialization"]["velocity"][1].as<double>(),
                data["imu_initialization"]["velocity"][2].as<double>());
            init.omega_bias = Eigen::Vector3d(
                data["imu_initialization"]["omega_bias"][0].as<double>(),
                data["imu_initialization"]["omega_bias"][1].as<double>(),
                data["imu_initialization"]["omega_bias"][2].as<double>());
            init.acc_bias = Eigen::Vector3d(
                data["imu_initialization"]["acc_bias"][0].as<double>(),
                data["imu_initialization"]["acc_bias"][1].as<double>(),
                data["imu_initialization"]["acc_bias"][2].as<double>());
            init.velocity_sigma = data["imu_initialization"]["velocity_sigma"].as<double>();
            init.omega_bias_sigma = data["imu_initialization"]["omega_bias_sigma"].as<double>();
            init.acc_bias_sigma = data["imu_initialization"]["acc_bias_sigma"].as<double>();
        }
        else
        {
            LOG(FATAL) << "Could not load IMU initialization from file";
        }
        return init;
    }

    bool ChannelImu::GetAngularVelocity(
        double timestamp,
        Eigen::Vector3d &omega) const
    {
        timestamp -= imu_calib_.delay_imu_cam; // correct for camera delay
        double min_delta = imu_calib_.max_imu_delta_t;
        for (ImuMeasurements::const_iterator it = measurements_.begin(); it != measurements_.end(); ++it)
        {
            const double dt = std::fabs(it->timestamp_ - timestamp);
            if (dt < min_delta)
            {
                omega = it->angular_velocity_;
                min_delta = dt;
            }
        }
        if (min_delta < imu_calib_.max_imu_delta_t)
            return true;
        return false;
    }

    // compute gravity direction using one sample of imu data
    bool ChannelImu::GetInitialAttitude(
        double timestamp,
        Quaternion &R_imu_world) const
    {
        ImuMeasurement m;
        if (!GetClosestMeasurement(timestamp, m))
        {
            LOG(WARNING) << "ChannelImu: Could not get initial attitude. No measurements!";
            return false;
        }

        // Set initial coordinate frame based on gravity direction.
        const Eigen::Vector3d &g = m.linear_acceleration_;
        const Eigen::Vector3d z = g.normalized(); // imu measures positive-z when static

        // TODO: make sure z != -1,0,0
        Eigen::Vector3d p(1, 0, 0);
        Eigen::Vector3d p_alternative(0, 1, 0);
        if (std::fabs(z.dot(p)) > std::fabs(z.dot(p_alternative)))
            p = p_alternative;
        Eigen::Vector3d y = z.cross(p); // make sure gravity is not in x direction
        y.normalize();
        const Eigen::Vector3d x = y.cross(z);
        Eigen::Matrix3d C_imu_world; // world unit vectors in imu coordinates
        C_imu_world.col(0) = x;
        C_imu_world.col(1) = y;
        C_imu_world.col(2) = z;

        VLOG(3) << "Initial Rotation = " << C_imu_world;
        //std::cout << "Initial Rotation = " << C_imu_world;
        R_imu_world = Quaternion(C_imu_world);
        return true;
    }

    // compute gravity direction by averaging imu data
    bool ChannelImu::GetInitialAttitude(
        Quaternion& R_imu_world) const
    {
        int max_imu_cnt = 100;
        ulock_t lock(measurements_mut_);
        if((int)measurements_.size() < max_imu_cnt)
        {
            LOG(WARNING) << "ImuHandler: don't have enough imu measurements!";
            return false;
        }
        
        Eigen::Vector3d mean_acc = Eigen::Vector3d::Zero();
        Eigen::Vector3d var_acc = Eigen::Vector3d::Zero();
        Eigen::Vector3d std_acc = Eigen::Vector3d::Zero();

        for(int acc_cnt = 0; acc_cnt < max_imu_cnt; ++acc_cnt)
        {
            const ImuMeasurement &m = measurements_[acc_cnt];
            mean_acc += m.linear_acceleration_;
        }  
        mean_acc /= static_cast<double>(max_imu_cnt);
        
        for(int acc_cnt = 0; acc_cnt < max_imu_cnt; ++acc_cnt)
        {
            const ImuMeasurement &m = measurements_[acc_cnt];
            var_acc += (m.linear_acceleration_ - mean_acc).cwiseAbs2();
        }
        var_acc /= static_cast<double>(max_imu_cnt-1);
        std_acc  = var_acc.cwiseSqrt();

        int refine_cnt = 0;
        Eigen::Vector3d mean_acc_refine;
        for(int acc_cnt = 0; acc_cnt < max_imu_cnt; ++acc_cnt)
        {
            const ImuMeasurement &m = measurements_[acc_cnt];
            if((m.linear_acceleration_-mean_acc).norm() < 3.0*std_acc.norm())
            {
                mean_acc_refine += m.linear_acceleration_;
                ++refine_cnt;
            }
        }

        if(refine_cnt < 10)
            return false;

        mean_acc_refine /= refine_cnt;
        
        const Eigen::Vector3d& g = mean_acc_refine;
        const Eigen::Vector3d z = g.normalized();
        // TODO: make sure z != -1,0,0
        Eigen::Vector3d p(1,0,0);
        Eigen::Vector3d p_alternative(0,1,0);
        if(std::fabs(z.dot(p)) > std::fabs(z.dot(p_alternative)))
          p = p_alternative;
        Eigen::Vector3d y = z.cross(p);
        y.normalize();
        const Eigen::Vector3d x = y.cross(z);
        Eigen::Matrix3d C_imu_world;
        C_imu_world.col(0) = x;
        C_imu_world.col(1) = y;
        C_imu_world.col(2) = z;

        VLOG(3) << "Initial Rotation = " << C_imu_world;
        // std::cout << "Initial Rotation = \n" 
        //     << Eigen::Quaterniond(C_imu_world).coeffs().transpose() << "\n";

        R_imu_world = Quaternion(C_imu_world);
        return true;
    }
    
    bool ChannelImu::getInitialAttitude(
        Quaternion& R_imu_world) const
    {
        int max_imu_cnt = 100;
        ulock_t lock(measurements_mut_);
        if((int)measurements_.size() < max_imu_cnt)
        {
            LOG(WARNING) << "ImuHandler: don't have enough imu measurements!";
            return false;
        }

        double max_acc_std = 0.5; 
        
        Eigen::Vector3d std_acc = Eigen::Vector3d::Zero();
        Eigen::Vector3d mean_acc = Eigen::Vector3d::Zero();

        int meas_size = measurements_.size();
        for(int acc_cnt = 0; acc_cnt < max_imu_cnt; ++acc_cnt)
        {
            const ImuMeasurement &m = measurements_[acc_cnt];
            mean_acc += m.linear_acceleration_;
        }  
        mean_acc /= static_cast<double>(max_imu_cnt);
        
        for(int acc_cnt = 0; acc_cnt < max_imu_cnt; ++acc_cnt)
        {
            const ImuMeasurement &m = measurements_[acc_cnt];
            std_acc += (m.linear_acceleration_ - mean_acc).cwiseAbs2();
        }
        std_acc /= static_cast<double>(max_imu_cnt);
        
        if (std_acc.norm() > max_acc_std || mean_acc.norm() == 0 || !std::isfinite(mean_acc.norm())) 
        {
            // std::cout << "[Initialize]: Initializaion failed: \n" << std::fixed 
            //       << "Mean acc data: " << mean_acc.transpose() << "\n"
            //       << "std acc data: " << std_acc.transpose() << "\n"
            //       << "std acc norm: " << std_acc.norm() << "\n";  
            return false;
        }
        // else 
            // std::cout << "[Initialize]: Initializaion success: \n" << std::fixed 
            //           << "Mean acc data: " << mean_acc.transpose() << "\n"
            //           << "std acc data: " << std_acc.transpose() << "\n"
            //           << "std acc norm: " << std_acc.norm() << "\n"; 

        const Eigen::Vector3d& g = mean_acc;
        const Eigen::Vector3d z = g.normalized();
        // TODO: make sure z != -1,0,0
        Eigen::Vector3d p(1,0,0);
        Eigen::Vector3d p_alternative(0,1,0);
        if(std::fabs(z.dot(p)) > std::fabs(z.dot(p_alternative)))
          p = p_alternative;
        Eigen::Vector3d y = z.cross(p);
        y.normalize();
        const Eigen::Vector3d x = y.cross(z);
        Eigen::Matrix3d C_imu_world;
        C_imu_world.col(0) = x;
        C_imu_world.col(1) = y;
        C_imu_world.col(2) = z;

        VLOG(3) << "Initial Rotation = " << C_imu_world;
        // std::cout << "Initial Rotation = \n" 
        //     << Eigen::Quaterniond(C_imu_world).coeffs().transpose() << "\n";

        R_imu_world = Quaternion(C_imu_world);
        return true;
    }

    void ChannelImu::reset()
    {
        ulock_t lock(measurements_mut_);
        measurements_.clear();
        temporal_imu_window_.clear();
    }

    IMUTemporalStatus ChannelImu::CheckTemporalStatus(const double time_sec)
    {
        IMUTemporalStatus res = IMUTemporalStatus::kMoving;

        if (!options_.temporal_stationary_check)
        {
            CHECK_EQ(temporal_imu_window_.size(), 0u);
            LOG(WARNING) << "Stationary check is not enabled. Will assume moving.";
            return res;
        }

        // do we have the IMU up to this time?
        if (temporal_imu_window_.empty() ||
            temporal_imu_window_.front().timestamp_ < time_sec)
        {
            return IMUTemporalStatus::kUnkown;
        }
        // check whether we have enough IMU measurements before
        int start_idx = -1;
        int end_idx = -1;
        for (size_t idx = 0; idx < temporal_imu_window_.size(); idx++)
        {
            if (start_idx == -1 &&
                temporal_imu_window_[idx].timestamp_ < time_sec)
            {
                // we know the first is not the start point
                CHECK_GT(idx, 0u);
                start_idx = idx - 1;
                continue;
            }

            if (start_idx != 1 &&
                (time_sec - temporal_imu_window_[idx].timestamp_ > options_.temporal_window_length_sec_))
            {
                end_idx = idx;
                break;
            }
        }
        if (end_idx == -1 || start_idx == -1)
        {
            return IMUTemporalStatus::kUnkown;
        }
        // check the status by standard deviation
        const double sqrt_dt = std::sqrt(1.0 / imu_calib_.imu_rate);
        std::vector<double> gyr_x(end_idx - start_idx + 1);
        std::vector<double> gyr_y(end_idx - start_idx + 1);
        std::vector<double> gyr_z(end_idx - start_idx + 1);
        std::vector<double> acc_x(end_idx - start_idx + 1);
        std::vector<double> acc_y(end_idx - start_idx + 1);
        std::vector<double> acc_z(end_idx - start_idx + 1);
        size_t idx = 0;
        for (size_t midx = start_idx; midx <= static_cast<size_t>(end_idx);
             midx++)
        {
            const ImuMeasurement &m = temporal_imu_window_[midx];
            gyr_x[idx] = m.angular_velocity_.x();
            gyr_y[idx] = m.angular_velocity_.y();
            gyr_z[idx] = m.angular_velocity_.z();
            acc_x[idx] = m.linear_acceleration_.x();
            acc_y[idx] = m.linear_acceleration_.y();
            acc_z[idx] = m.linear_acceleration_.z();
            idx++;
        }
        std::array<double, 3> gyr_std{0.0, 0.0, 0.0};
        std::array<double, 3> acc_std{0.0, 0.0, 0.0};
        gyr_std[0] = stdVec(gyr_x) * sqrt_dt;
        gyr_std[1] = stdVec(gyr_y) * sqrt_dt;
        gyr_std[2] = stdVec(gyr_z) * sqrt_dt;
        acc_std[0] = stdVec(acc_x) * sqrt_dt;
        acc_std[1] = stdVec(acc_y) * sqrt_dt;
        acc_std[2] = stdVec(acc_z) * sqrt_dt;

        bool stationary = true;
        for (size_t idx = 0; idx < 3; idx++)
        {
            stationary &= (gyr_std[idx] < options_.stationary_gyr_sigma_thresh_);
            stationary &= (acc_std[idx] < options_.stationary_acc_sigma_thresh_);
        }

        // remove up to the used ones to make sure we still have enough
        temporal_imu_window_.erase(temporal_imu_window_.begin() + end_idx,
                                   temporal_imu_window_.end());

        return stationary ? IMUTemporalStatus::kStationary : IMUTemporalStatus::kMoving;
    }

    bool ChannelImu::WaitTill(const double img_timestamp_sec,
                              const double timeout_sec)
    {
        vk::Timer wait_time;
        wait_time.start();
        while (this->GetLatestTimestamp() <
               img_timestamp_sec - this->imu_calib_.delay_imu_cam)
        {
            if (wait_time.stop() > timeout_sec)
            {
                LOG(ERROR) << "Did not get IMU measurements";
                return false;
            }
            wait_time.resume();
            // wait
            VLOG(50) << "Waiting for imu measurements.";
        }

        return true;
    }

} // namespace mivins
