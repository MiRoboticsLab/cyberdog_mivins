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

#include <svo/data_align.h>

namespace mivins
{

DataAlign::DataAlign()
{
  first_img_for_imu_ = true;
  interpolation_ptr_ = std::make_shared<loose_couple::Interpolation>();
}

DataAlign::~DataAlign()
{

}

void DataAlign::reset()
{
  first_img_for_imu_ = true;

  imu_queue_.clear();
  matched_img_imus_.clear();
  matched_img_imu_ts_.clear();
  img_ts_queue_for_imu_.clear();
  
  odom_queue_.clear();
  matched_img_odoms_.clear();
  matched_img_odom_ts_.clear();
  img_ts_queue_for_odom_.clear();
}

bool DataAlign::getAlignedOdom(const double timestamp, Eigen::Matrix4d &odom_pose)
{
  odom_pose.setIdentity();

  if(!matched_img_odoms_.count(timestamp))
  {
    printf("Couldn't find corresponding odome data\n");
    if(!matched_img_odoms_.empty())
    {
      printf("last image ts: %.9lf, query image ts: %.9lf\n", 
              matched_img_odom_ts_.back(), timestamp);
    }
    return false;
  }

  loose_couple::OdomData odom_data = matched_img_odoms_[timestamp];
  odom_pose.block<3, 3>(0, 0) = odom_data.orientation_.toRotationMatrix();
  odom_pose.block<3, 1>(0, 3) = odom_data.position_;
  return true;
}

bool DataAlign::getLatestAlignedOdom(double &timestamp, Eigen::Matrix4d &odom_pose)
{
  if(matched_img_odoms_.empty())
  {
    printf("matched_img_odoms_ is empty!\n");
    return false;
  }

  odom_pose.setIdentity();
  timestamp = matched_img_odom_ts_.back();

  loose_couple::OdomData odom_data = matched_img_odoms_[timestamp];
  odom_pose.block<3, 3>(0, 0) = odom_data.orientation_.toRotationMatrix();
  odom_pose.block<3, 1>(0, 3) = odom_data.position_;

  return true;
}

bool DataAlign::imuAvailable(const double img_ts)
{
  if(!imu_queue_.empty() && img_ts <= imu_queue_.back().timestamp_)
    return true;

  return false;
}

bool DataAlign::odomAvailable(const double img_ts)
{
  if(!odom_queue_.empty() && img_ts <= odom_queue_.back().timestamp_)
    return true;

  return false;
}

void DataAlign::inputImu(const double timestamp,
      const Eigen::Vector3d &angular_velocity,
      const Eigen::Vector3d &linear_acceleration)
{
  loose_couple::ImuData imu_data(timestamp, angular_velocity, linear_acceleration);

  imu_mutex_.lock();
  imu_queue_.push_back(imu_data);
  while(timestamp - imu_queue_.front().timestamp_ > 10.0f)
    imu_queue_.pop_front();
  imu_mutex_.unlock();
}

void DataAlign::inputOdom(const double timestamp, const Eigen::Quaterniond &orientation,
  const Eigen::Vector3d &position, const Eigen::Vector3d &linear_velocity, 
  const Eigen::Vector3d &angular_velocity)
{
  // inputRecalOdom(timestamp, orientation, position, linear_velocity, angular_velocity);
  // return;

  loose_couple::OdomData cur_odom(timestamp, orientation, position, 
                    linear_velocity, angular_velocity);

  if(odom_cnt_ == 0)
    prv_odom_ = cur_odom;

  ++odom_cnt_;

  odom_mutex_.lock();
  odom_queue_.push_back(cur_odom);
  while(timestamp - odom_queue_.front().timestamp_ > 10.0f)
    odom_queue_.pop_front();
  odom_mutex_.unlock();
}

void DataAlign::align(const double img_ts)
{
  bool res_imu = false, res_odom = false;
  // res_imu = imageAndImuAlign(img_ts);
  res_odom = imageAndOdomAlign(img_ts);
}

void DataAlign::getImuMeasurements(
    const double ts0, const double ts1, 
    loose_couple::ImuDatas &imu_datas)
{
  imu_datas.clear();

  if(!imu_queue_.empty() 
    && imu_queue_.front().timestamp_ <= ts0 
    && imu_queue_.back().timestamp_ >= ts1)
  {
    int big_idx = -1;
    int small_idx = -1;
    imu_mutex_.lock();
    for(int i = 0; i < (int)imu_queue_.size(); ++i)
    {
      loose_couple::ImuData imu_data = imu_queue_[i];
      double imu_ts = imu_data.timestamp_;
      if(imu_data.timestamp_ < ts0)
        small_idx = i;
      else if(imu_ts >= ts0 && imu_ts <= ts1)
        imu_datas.push_back(imu_queue_[i]);
      else
      {
        big_idx = i;
        break;
      }
    }

    if(small_idx != -1)
    {
      loose_couple::ImuData imu_data0 = imu_queue_[small_idx];
      loose_couple::ImuData imu_data1 = imu_queue_[small_idx + 1];
      loose_couple::ImuData inter_imu_data = imuInterpolation(ts0, imu_data0, imu_data1);
      imu_datas.push_front(inter_imu_data);
    }

    if(big_idx != -1)
    {
      loose_couple::ImuData imu_data0 = imu_queue_[big_idx - 1];
      loose_couple::ImuData imu_data1 = imu_queue_[big_idx];
      loose_couple::ImuData inter_imu_data = imuInterpolation(ts1, imu_data0, imu_data1);
      imu_datas.push_back(inter_imu_data);
    }

    imu_mutex_.unlock();
  }
}

loose_couple::ImuData DataAlign::imuInterpolation(
    const double inter_ts,
    const loose_couple::ImuData &imu_data1, 
    const loose_couple::ImuData &imu_data2)
{
    double ts1 = imu_data1.timestamp_;
    Eigen::Vector3d gyr1 = imu_data1.angular_velocity_;
    Eigen::Vector3d acc1 = imu_data1.linear_acceleration_;

    double ts2 = imu_data2.timestamp_;
    Eigen::Vector3d gyr2 = imu_data2.angular_velocity_;
    Eigen::Vector3d acc2 = imu_data2.linear_acceleration_;
    
    Eigen::Vector3d inter_acc = Eigen::Vector3d::Zero();
    interpolation_ptr_->linearInterpolation(ts1, acc1, ts2, acc2, inter_ts, inter_acc);

    Eigen::Vector3d inter_gyr = Eigen::Vector3d::Zero();
    interpolation_ptr_->linearInterpolation(ts1, gyr1, ts2, gyr2, inter_ts, inter_gyr);
    
    return loose_couple::ImuData(inter_ts, inter_gyr, inter_acc);
}

bool DataAlign::imageAndImuAlign(const double img_ts)
{
  if(matched_img_imus_.count(img_ts))
    return true;

  img_ts_queue_for_imu_.push_back(img_ts);

  if(!imuAvailable(img_ts_queue_for_imu_.front()))
    return false;

  imu_mutex_.lock();

  while(!img_ts_queue_for_imu_.empty())
  {
    double query_ts = img_ts_queue_for_imu_.front();
    if(!imuAvailable(query_ts))
      break;

    while(!imu_queue_.empty())
    {    
      if(imu_queue_.back().timestamp_ < query_ts)
        break;
      
      if(first_img_for_imu_)
      {
        first_img_for_imu_ = false;

        while(imu_queue_.front().timestamp_ < query_ts)
          imu_queue_.pop_front();
      }
      else
      {
        while(imu_queue_.front().timestamp_ < query_ts)
        {
          loose_couple::ImuData imu_data = imu_queue_.front();
          matched_img_imus_[query_ts].push_back(imu_data);
          imu_queue_.pop_front();
        }

        if(!matched_img_imus_[query_ts].empty() && !imu_queue_.empty())
        {
          loose_couple::ImuData imu_data1 = matched_img_imus_[query_ts].back();
          loose_couple::ImuData imu_data2 = imu_queue_.front();

          loose_couple::ImuData query_imu = imuInterpolation(query_ts, imu_data1, imu_data2);
          
          matched_img_imus_[query_ts].push_back(query_imu);
          imu_queue_.push_front(query_imu);
        }
      }

      img_ts_queue_for_imu_.pop_front();
      break;
    }
  }

  imu_mutex_.unlock();

  return img_ts_queue_for_imu_.empty();
}

bool DataAlign::imageAndOdomAlign(const double img_ts)
{
  if(matched_img_odoms_.count(img_ts))
    return true;

  img_ts_queue_for_odom_.push_back(img_ts);

  if(!odomAvailable(img_ts_queue_for_odom_.front()))
    return false;

  odom_mutex_.lock();

  while(!img_ts_queue_for_odom_.empty())
  {
    loose_couple::OdomData query_odom;
    double query_ts = img_ts_queue_for_odom_.front();

    while(!odom_queue_.empty())
    {    
      loose_couple::OdomData cur_odom = odom_queue_.front();

      if(query_ts < prv_odom_.timestamp_)
      {
        img_ts_queue_for_odom_.pop_front();
        break;
      }
      else if(query_ts >= prv_odom_.timestamp_ && query_ts <= cur_odom.timestamp_)
      {
        // printf("%.9lf, %.9lf, %.9lf\n", prv_odom_.timestamp_, query_ts, cur_odom.timestamp_);
        odomInterpolation(prv_odom_, cur_odom, query_ts, query_odom);
        matched_img_odoms_[query_ts] = query_odom;
        matched_img_odom_ts_.push_back(query_ts);

        img_ts_queue_for_odom_.pop_front();

        odom_queue_.pop_front();
        prv_odom_ = cur_odom;
        break;
      }
      else if(query_ts > cur_odom.timestamp_)
      {
        odom_queue_.pop_front();
        prv_odom_ = cur_odom;
      }
    }

    if(odom_queue_.empty())
      break;
  }

  odom_mutex_.unlock();

  // std::cout << matched_img_odom_ts_.size() << " --- " << matched_img_odoms_.size() << "\n";
  // assert(matched_img_odom_ts_.size() == matched_img_odoms_.size());
  /*
  if(matched_img_odom_ts_.size() != matched_img_odoms_.size())
  {
    std::cout << "Different size\n";
    std::cout << matched_img_odom_ts_.size() << " --- " << matched_img_odoms_.size() << "\n";
  }
  */

  return img_ts_queue_for_odom_.empty();
}

void DataAlign::odomInterpolation(
      const loose_couple::OdomData &odom1, const loose_couple::OdomData &odom2, 
      const double ts, loose_couple::OdomData &odom)
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();

  interpolation_ptr_->poseInterpolation(odom1.timestamp_, odom1.orientation_, odom1.position_,
                                 odom2.timestamp_, odom2.orientation_, odom2.position_,
                                 ts, orientation, position);
  
  interpolation_ptr_->linearInterpolation(odom1.timestamp_, odom1.linear_velocity_,
                                   odom2.timestamp_, odom2.linear_velocity_,
                                   ts, linear_velocity);
  
  interpolation_ptr_->linearInterpolation(odom1.timestamp_, odom1.angular_velocity_,
                                   odom2.timestamp_, odom2.angular_velocity_,
                                   ts, angular_velocity);

  odom = loose_couple::OdomData(ts, orientation, position, linear_velocity, angular_velocity);
}


void DataAlign::inputRecalOdom(
  const double timestamp, const Eigen::Quaterniond &orientation,
  const Eigen::Vector3d &position, const Eigen::Vector3d &linear_velocity, 
  const Eigen::Vector3d &angular_velocity)
{
  static loose_couple::OdomData prv_odom_org, prv_odom_cal, cur_odom_cal;

  loose_couple::OdomData cur_odom_org(timestamp, orientation, position, 
                    linear_velocity, angular_velocity);

  ++odom_cnt_;

  if(odom_cnt_ == 1)
    prv_odom_ = cur_odom_org;
  else if(odom_cnt_ == 2)
  {
    double delta_ts = cur_odom_org.timestamp_ - prv_odom_org.timestamp_;

    Eigen::Vector3d delta_p = prv_odom_org.orientation_.toRotationMatrix().transpose()
                              * (cur_odom_org.position_ - prv_odom_org.position_);
    double dist = (prv_odom_org.linear_velocity_ * delta_ts).norm();
    delta_p = delta_p.normalized() * dist;
    Eigen::Vector3d linear_velocity = delta_p / delta_ts;

    Eigen::Quaterniond delta_q = prv_odom_org.orientation_.inverse() 
                                  * cur_odom_org.orientation_;
    
    Eigen::Vector3d delta_theta = 2 * delta_q.normalized().vec();
    Eigen::Vector3d angular_velocity = delta_theta / delta_ts;

    cur_odom_cal = prv_odom_org;
    cur_odom_cal.linear_velocity_ = linear_velocity;
    cur_odom_cal.angular_velocity_ = angular_velocity;
  }
  else
    recalculateOdom(prv_odom_org, cur_odom_org, prv_odom_cal, cur_odom_cal);

  prv_odom_org = cur_odom_org;
  prv_odom_cal = cur_odom_cal;

  if(odom_cnt_ < 2)
    return;

  // fprintf(fp_log_, "%.9lf %.9lf %.9lf\n", cur_odom_cal.position_.x(), 
  //           cur_odom_cal.position_.y(), cur_odom_cal.position_.z());

  odom_mutex_.lock();
  odom_queue_.push_back(cur_odom_cal);
  odom_mutex_.unlock();
}

void DataAlign::recalculateOdom(
  const loose_couple::OdomData &prv_odom_org, const loose_couple::OdomData &cur_odom_org, 
  const loose_couple::OdomData &prv_odom_cal, loose_couple::OdomData &cur_odom_cal)
{
  double delta_ts = cur_odom_org.timestamp_ - prv_odom_org.timestamp_;

  Eigen::Vector3d delta_p = prv_odom_org.orientation_.toRotationMatrix().transpose()
                            * (cur_odom_org.position_ - prv_odom_org.position_);
  double dist = (prv_odom_org.linear_velocity_ * delta_ts).norm();
  delta_p = delta_p.normalized() * dist;
  Eigen::Vector3d linear_velocity = delta_p / delta_ts;

  Eigen::Quaterniond delta_q = prv_odom_org.orientation_.inverse() 
                                * cur_odom_org.orientation_;
  
  Eigen::Vector3d delta_theta = 2 * delta_q.normalized().vec();
  Eigen::Vector3d angular_velocity = delta_theta / delta_ts; 

  {
    delta_ts = prv_odom_org.timestamp_ - prv_odom_cal.timestamp_;
    delta_theta = prv_odom_cal.angular_velocity_ * delta_ts;
    delta_q = Eigen::Quaterniond(1.0f, delta_theta.x() * 0.5f, 
                  delta_theta.y() * 0.5f, delta_theta.z() * 0.5f);
    Eigen::Quaterniond orientation = prv_odom_cal.orientation_ * delta_q.normalized();
    Eigen::Vector3d position = prv_odom_cal.position_ + prv_odom_cal.orientation_ * 
                  (prv_odom_cal.linear_velocity_ * delta_ts);

    cur_odom_cal.timestamp_ = prv_odom_org.timestamp_;
    
    cur_odom_cal.position_ = position;
    cur_odom_cal.orientation_ = orientation;

    cur_odom_cal.linear_velocity_ = linear_velocity;
    cur_odom_cal.angular_velocity_ = angular_velocity;
  }
}

}
