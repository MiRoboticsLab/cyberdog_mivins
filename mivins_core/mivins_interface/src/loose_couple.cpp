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

#include <svo/loose_couple.h>

namespace mivins
{

LooseCouple::LooseCouple(
    const LooseCoupleOptions &options, 
    const Eigen::Matrix4d &Tbo)
{
  options_ = options;
  options_.print();

  Tbo_ = Tbo;
  Tob_ = inverse(Tbo);

  gyr_bias_.setZero();

  global_pose_.setIdentity();

  first_init_ = true;

  is_abnormal_ = false;

  use_vio_pose_ = true;

  first_init_ts_ = 0.0f;
  
  interpolation_ptr_ = std::make_shared<loose_couple::Interpolation>();

  char log_file[256];
  snprintf(log_file, sizeof(log_file), "%s/debug/%s", 
          getenv("HOME"), "loose_couple_log.txt");
  fp_log_ = fopen(log_file, "wb");
}

LooseCouple::~LooseCouple()
{
  kf_vio_data_.clear();

  global_data_.clear();

  gyr_bias_.setZero();
}

void LooseCouple::reset()
{
  ab_cnt_ = 0;
  is_abnormal_ = false;
  use_vio_pose_ = true;
  // TODO: Clear parameters
}

void LooseCouple::initWithOdom(const bool init_with_odom)
{
  init_with_odom_ = init_with_odom;
}

void LooseCouple::setAlignHandler(const DataAlign::Ptr &align_ptr)
{
  if(align_ptr_)
    return;

  align_ptr_ = align_ptr;
}

void LooseCouple::setGyrBias(const Eigen::Vector3d &gyr_bias)
{
  gyr_bias_ = gyr_bias;
}

bool LooseCouple::isAbnormal()
{
  return is_abnormal_;
}

Eigen::Matrix4d LooseCouple::getFusedPose()
{
  return global_pose_;
}

Eigen::Matrix4d LooseCouple::getFusedPose(const double timestamp)
{
  Eigen::Matrix4d fused_pose = getFusedPoseWithOdom(timestamp);
  return fused_pose;
}

void LooseCouple::addFrameBundle(
    const double timestamp, const Eigen::Matrix4d &T_w_b, 
    const bool is_keyframe, const bool vio_state, 
    const bool is_backend_ok)
{
  cur_vio_ = loose_couple::VioData(timestamp, is_keyframe, T_w_b);
  if(is_keyframe)
  {
    kf_vio_data_.push_back(cur_vio_);
    
    while(!kf_vio_data_.empty())
    {
      const loose_couple::VioData &old_data = kf_vio_data_.front();
      const loose_couple::VioData &new_data = kf_vio_data_.back(); 
      double delta_ts = new_data.timestamp_ - old_data.timestamp_;

      if(delta_ts > options_.ab_backtrack_ts + 2.0f)
        kf_vio_data_.pop_front();
      else
        break;
    }

    while(!global_data_.empty())
    {
      const loose_couple::VioData &old_data = global_data_.front();
      const loose_couple::VioData &new_data = global_data_.back();
      double delta_ts = new_data.timestamp_ - old_data.timestamp_;
      if(delta_ts > options_.ab_backtrack_ts + 0.5f)
        global_data_.pop_front();
      else
        break;
    }
  }

  if(first_init_)
  {
    if(first_init_ts_ == 0.0f)
      first_init_ts_ = timestamp;

    const double delta_ts = timestamp - first_init_ts_;
    
    if(!vio_state && init_with_odom_ && delta_ts > options_.init_time_thresh)
    {
      if(align_ptr_->matched_img_odom_ts_.empty())
      {
        printf("For first initialize, couldn't find the corresponding odometry data\n"); 
        return ;
      }

      int last_idx = align_ptr_->matched_img_odom_ts_.size() - 1;
      global_pose_ = getMatchedPose(last_idx, true);
      use_vio_pose_ = false;
      first_init_ = false;
    }
    
    if(vio_state)
    {
      global_pose_ = T_w_b;
      use_vio_pose_ = true;
      first_init_ = false;
    }
  }
  else
  {   
    if(!is_backend_ok)
    {
      global_pose_ = abnormalProcess(timestamp);
      use_vio_pose_ = false;
      is_abnormal_ = true;
      ab_cnt_ = 0;
    }
    else
    {
      if(vio_state)
      {
        if(prv_state_ != vio_state)
        {
          // Eigen::Matrix4d delta_pose = getDeltaPoseWithOdom(true);
          // global_pose_ *= delta_pose;

          Eigen::Matrix4d fused_pose = getFusedPoseWithOdom(timestamp);
          global_pose_ = fused_pose;
          use_vio_pose_ = false;
        }
        else
        {
          int prv_ab_cnt_ = ab_cnt_;
          is_abnormal_ = abnormalDetection();

          if(!is_abnormal_)
          {
            if(prv_ab_cnt_ == ab_cnt_)
            {
              const Eigen::Matrix4d &prv_T_w_b = prv_vio_.T_w_b_;
              const Eigen::Matrix4d &cur_T_w_b = cur_vio_.T_w_b_;
              Eigen::Matrix4d delta_pose = inverse(prv_T_w_b) * cur_T_w_b;
              global_pose_ *= delta_pose;
              use_vio_pose_ = true;
            }
            else
            {
              Eigen::Matrix4d fused_pose = getFusedPoseWithOdom(timestamp);
              global_pose_ = fused_pose;
              use_vio_pose_ = false;
            }
          }
          else
          {
            global_pose_ = abnormalProcess(timestamp);
            use_vio_pose_ = false;
          }
        }
      }
      else if(!vio_state)
      {
        // Eigen::Matrix4d delta_pose = getDeltaPoseWithOdom(true);
        // global_pose_ *= delta_pose;
        Eigen::Matrix4d fused_pose = getFusedPoseWithOdom(timestamp);
        global_pose_ = fused_pose;
        use_vio_pose_ = false;
      }
    }
  }

  prv_vio_ = cur_vio_;
  prv_state_ = vio_state;

  loose_couple::VioData data(timestamp, is_keyframe, global_pose_);
  global_data_.push_back(data);
}

Eigen::Matrix4d LooseCouple::getFusedPoseWithOdom(const double timestamp)
{
  Eigen::Matrix4d fused_pose = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d delta_pose = Eigen::Matrix4d::Identity();

  fused_pose = global_pose_;

  if(align_ptr_->matched_img_odoms_.empty())
    return fused_pose;

  bool find_prv_odom = false;
  loose_couple::OdomData prv_odom;
  loose_couple::VioData global_data;
  if(align_ptr_->matched_img_odoms_.count(timestamp))
  {
    const loose_couple::OdomData cur_odom = align_ptr_->matched_img_odoms_[timestamp];
    for(int i = (int)global_data_.size() - 1; i >= 0; --i)
    {
      global_data = global_data_[i];
      if(!align_ptr_->matched_img_odoms_.count(global_data.timestamp_))
        continue;

      prv_odom = align_ptr_->matched_img_odoms_[global_data.timestamp_];

      delta_pose = getDeltaPoseWithOdom(prv_odom, cur_odom, false);

      delta_pose = Tbo_ * delta_pose * Tob_;

      find_prv_odom = true;
      
      break;
    }
  }
  else
  {
    for(int i = (int)global_data_.size() - 1; i >= 0; --i)
    {
      global_data = global_data_[i];
      if(!align_ptr_->matched_img_odoms_.count(global_data.timestamp_))
        continue;

      prv_odom = align_ptr_->matched_img_odoms_[global_data.timestamp_];
      double delta_ts = timestamp - prv_odom.timestamp_;
 
      Eigen::Vector3d delta_theta = prv_odom.angular_velocity_ * delta_ts;
      Eigen::Quaterniond delta_q(1.0f, delta_theta.x() * 0.5f, 
              delta_theta.y() * 0.5f, delta_theta.z() * 0.5f);
      Eigen::Vector3d delta_t = prv_odom.linear_velocity_ * delta_ts;

      delta_pose.block<3, 3>(0, 0) = delta_q.normalized().toRotationMatrix();
      delta_pose.block<3, 1>(0, 3) = delta_t;
      
      delta_pose = Tbo_ * delta_pose * Tob_;
        
      break;
    }
  }

  if(options_.use_imu_rot && find_prv_odom)
  {
    loose_couple::ImuDatas imu_datas;
    const double &cur_ts = timestamp;
    const double &prv_ts = prv_odom.timestamp_;
    align_ptr_->getImuMeasurements(prv_ts, cur_ts, imu_datas);
    if(!imu_datas.empty())
    {
      Eigen::Matrix3d delta_rot = getDeltaRotationWithImu(imu_datas);
      delta_pose.block<3, 3>(0, 0) = delta_rot;
    }
  }

  fused_pose = global_data.T_w_b_ * delta_pose;

  return fused_pose;
}

Eigen::Matrix3d LooseCouple::getDeltaRotationWithImu(
  const loose_couple::ImuDatas &imu_datas)
{
  Eigen::Matrix3d delta_rot = Eigen::Matrix3d::Identity();
  
  for(int i = 0; i < (int)imu_datas.size() - 1; ++i)
  {
    Eigen::Vector3d gyr0 = imu_datas[i].angular_velocity_;
    Eigen::Vector3d gyr1 = imu_datas[i + 1].angular_velocity_;
    Eigen::Vector3d gyr = 0.5 * (gyr0 + gyr1) - gyr_bias_;
    double delta_ts = imu_datas[i + 1].timestamp_ - imu_datas[i].timestamp_;
    Eigen::Vector3d delta_theta = delta_ts * gyr;
    Eigen::Quaterniond delta_q = Eigen::Quaterniond(1.0f, delta_theta.x() * 0.5f, 
                                  delta_theta.y() * 0.5f, delta_theta.z() * 0.5f);
    delta_q.normalize();
    delta_rot  = delta_rot * delta_q.toRotationMatrix();
  }

  return delta_rot;
}

Eigen::Matrix4d LooseCouple::getDeltaPoseWithOdom(
  const loose_couple::OdomData &odom_data1, 
  const loose_couple::OdomData &odom_data2, 
  const bool use_imu)
{
  Eigen::Matrix4d delta_pose = Eigen::Matrix4d::Identity();

  Eigen::Matrix4d T_w_o1 = Eigen::Matrix4d::Identity();
  T_w_o1.block<3, 3>(0, 0) = odom_data1.orientation_.toRotationMatrix();
  T_w_o1.block<3, 1>(0, 3) = odom_data1.position_;

  Eigen::Matrix4d T_w_o2 = Eigen::Matrix4d::Identity();
  T_w_o2.block<3, 3>(0, 0) = odom_data2.orientation_.toRotationMatrix();
  T_w_o2.block<3, 1>(0, 3) = odom_data2.position_;

  delta_pose = inverse(T_w_o1) * T_w_o2;

  // getDeltaPoseWithScale(odom_data1, odom_data2, delta_pose);

  if(use_imu)
    delta_pose = Tbo_ * delta_pose * Tob_;

  return delta_pose;
}

Eigen::Matrix4d LooseCouple::getDeltaPoseWithOdom(bool imu_pose)
{
  if(align_ptr_->matched_img_odom_ts_.size() < 2)
    return Eigen::Matrix4d::Identity();

  Eigen::Matrix4d delta_pose = Eigen::Matrix4d::Identity();

  int last_idx1 = align_ptr_->matched_img_odom_ts_.size() - 1;
  int last_idx2 = align_ptr_->matched_img_odom_ts_.size() - 2;
  Eigen::Matrix4d odom_T_w_b1 = getMatchedPose(last_idx1, imu_pose);
  Eigen::Matrix4d odom_T_w_b2 = getMatchedPose(last_idx2, imu_pose);

  delta_pose = inverse(odom_T_w_b2) * odom_T_w_b1;

  return delta_pose;
}

Eigen::Matrix4d LooseCouple::getMatchedPose(int i, bool imu_pose)
{
  double img_ts = align_ptr_->matched_img_odom_ts_[i];
  Eigen::Vector3d &t_w_o = align_ptr_->matched_img_odoms_[img_ts].position_;
  Eigen::Quaterniond &q_w_o = align_ptr_->matched_img_odoms_[img_ts].orientation_;

  Eigen::Matrix4d T_w_o = Eigen::Matrix4d::Identity();
  T_w_o.block<3, 3>(0, 0) = q_w_o.toRotationMatrix();
  T_w_o.block<3, 1>(0, 3) = t_w_o;

  if(!imu_pose)
    return T_w_o;

  Eigen::Matrix4d T_w_i = Eigen::Matrix4d::Identity();
  T_w_i = T_w_o * Tob_;

  return T_w_i;
}

Eigen::Matrix4d LooseCouple::getMatchedPose(double ts, bool imu_pose)
{
  Eigen::Vector3d &t_w_o = align_ptr_->matched_img_odoms_[ts].position_;
  Eigen::Quaterniond &q_w_o = align_ptr_->matched_img_odoms_[ts].orientation_;

  Eigen::Matrix4d T_w_o = Eigen::Matrix4d::Identity();
  T_w_o.block<3, 3>(0, 0) = q_w_o.toRotationMatrix();
  T_w_o.block<3, 1>(0, 3) = t_w_o;

  if(!imu_pose)
    return T_w_o;

  Eigen::Matrix4d T_w_i = Eigen::Matrix4d::Identity();
  T_w_i = T_w_o * Tob_;

  return T_w_i;
}

void LooseCouple::getDeltaPoseWithScale(const loose_couple::OdomData &odom_data1, 
          const loose_couple::OdomData &odom_data2, Eigen::Matrix4d &delta_pose)
{
  double delta_ts = odom_data2.timestamp_ - odom_data1.timestamp_;
  Eigen::Vector3d delta_p = delta_pose.block<3, 1>(0, 3);

  assert(delta_ts >= 0);

  double dist = (odom_data1.linear_velocity_ * delta_ts).norm();
  delta_p = delta_p.normalized() * dist;

  delta_pose.block<3, 1>(0, 3) = delta_p;
}

bool LooseCouple::abnormalDetection()
{
  if(!cur_vio_.is_keyframe_)
    return false;

  int buf_size = kf_vio_data_.size();
  if(buf_size < 2)
    return false;

  const loose_couple::VioData &cur_vio = kf_vio_data_[buf_size - 1];
  const loose_couple::VioData &prv_vio = kf_vio_data_[buf_size - 2];

  const double cur_vio_ts = cur_vio.timestamp_;
  const double prv_vio_ts = prv_vio.timestamp_;

  if(!align_ptr_->matched_img_odoms_.count(prv_vio_ts) 
    || !align_ptr_->matched_img_odoms_.count(cur_vio_ts))
    return false;

  // Calculate the relative pose of two adjacent frames with vio pose(cur odom->prv odom)
  const Eigen::Matrix4d cur_vio_pose = cur_vio.T_w_b_ * Tbo_;
  const Eigen::Matrix4d prv_vio_pose = prv_vio.T_w_b_ * Tbo_;
  Eigen::Matrix4d T_prv_cur1 = inverse(prv_vio_pose) * cur_vio_pose;

  // Calculate the relative pose of two adjacent frames with odom pose(cur odom->prv odom)
  const Eigen::Matrix4d cur_odom_pose = getMatchedPose(cur_vio_ts, false);
  const Eigen::Matrix4d prv_odom_pose = getMatchedPose(prv_vio_ts, false);
  Eigen::Matrix4d T_prv_cur2 = inverse(prv_odom_pose) * cur_odom_pose;

  {
    const loose_couple::OdomData &cur_odom = align_ptr_->matched_img_odoms_[cur_vio_ts];
    const loose_couple::OdomData &prv_odom = align_ptr_->matched_img_odoms_[prv_vio_ts];
    getDeltaPoseWithScale(prv_odom, cur_odom, T_prv_cur2);
  }

  Eigen::Matrix3d R_prv_cur1 = T_prv_cur1.block<3, 3>(0, 0);
  Eigen::Matrix3d R_prv_cur2 = T_prv_cur2.block<3, 3>(0, 0);
  Eigen::Vector3d vio_ypr = interpolation_ptr_->R2ypr(R_prv_cur1, true);
  Eigen::Vector3d odom_ypr = interpolation_ptr_->R2ypr(R_prv_cur2, true);
  float vio_yaw = vio_ypr(0) < 0.0 ? vio_ypr(0) + 360.0 : vio_ypr(0);
  float odom_yaw = odom_ypr(0) < 0.0 ? odom_ypr(0) + 360.0 : odom_ypr[0];
  float delta_yaw = std::abs(vio_yaw - odom_yaw);
  delta_yaw = delta_yaw > 180.0 ? 360.0 - delta_yaw : delta_yaw;

  Eigen::Vector3d t_prv_cur1 = T_prv_cur1.block<3, 1>(0, 3);
  Eigen::Vector3d t_prv_cur2 = T_prv_cur2.block<3, 1>(0, 3);
  float vio_dist = t_prv_cur1.norm();
  float odom_dist = t_prv_cur2.norm();
  float dist_ratio = vio_dist / odom_dist;

  t_prv_cur1.normalize();
  t_prv_cur2.normalize();
  double cos_trans_direction = t_prv_cur1.transpose() * t_prv_cur2;

  std::cout << "delta_yaw: " << delta_yaw << "; "
            << "dist_ratio: " << dist_ratio << "; ";
  std::cout << "vio dist: " << vio_dist << "; "
            << "odom dist: " << odom_dist << "; ";
  std::cout << "cos_trans_direction: " << acos(cos_trans_direction) * 180 * M_1_PI << "; ";

  // fprintf(fp_log_, "delta yaw: %.3f\t dist ratio: %.3f\t", delta_yaw, dist_ratio);
  // fprintf(fp_log_, "vio dist: %.3f\t odom dist: %.3f\t", vio_dist, odom_dist);

  if( (std::isfinite(delta_yaw) && delta_yaw > options_.ab_delta_yaw) || 
      (std::isfinite(dist_ratio) && dist_ratio > options_.ab_dist_ratio) ||
      (odom_dist > 0.05 && cos_trans_direction < cos(options_.ab_direct_angle))
    )
  {
    ++ab_cnt_;
    // fprintf(fp_log_, "abnormal cnt: %d\n", ab_cnt_);
    std::cout <<  "abnormal cnt: " << ab_cnt_ << "\n"; 
    if(ab_cnt_ >= options_.ab_continuous_cnt)
    {
      ab_cnt_ = 0;
      return true;
    }
    return false;
  }

  ab_cnt_ = 0;
  // fprintf(fp_log_, "abnormal cnt: %d\n", ab_cnt_);
  // std::cout <<  "abnormal cnt: " << ab_cnt_ << "\n"; 
  return false;
}

Eigen::Matrix4d LooseCouple::abnormalProcess(const double timestamp)
{
  if(global_data_.empty() || align_ptr_->matched_img_odom_ts_.empty())
  {
    // TODO: This wouldn't happend
    return global_pose_; 
  }

  if(1)
  {
    std::deque<loose_couple::OdomData> prv_odom_datas;

    loose_couple::OdomData latest_odom;
    double latest_img_ts = align_ptr_->matched_img_odom_ts_.back();
    if(latest_img_ts > global_data_.back().timestamp_)
    {
      latest_odom = align_ptr_->matched_img_odoms_[latest_img_ts];
      prv_odom_datas.push_front(latest_odom);
    }

    Eigen::Matrix4d kf_T_w_b = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d kf_T_w_o = Eigen::Matrix4d::Identity();
    loose_couple::OdomData prv_odom;
    for(int i = (int)global_data_.size() - 1; i >= 0; --i)
    {
      const loose_couple::VioData &global_data = global_data_[i];
      double prv_timestamp = global_data.timestamp_;

      if(!align_ptr_->matched_img_odoms_.count(prv_timestamp))
        continue;

      prv_odom = align_ptr_->matched_img_odoms_[prv_timestamp];
      prv_odom_datas.push_front(prv_odom);
      kf_T_w_b = global_data.T_w_b_;

      if(!global_data.is_keyframe_)
        continue;
      
      kf_T_w_b = global_data.T_w_b_;

      double delta_ts = timestamp - prv_timestamp;
      if(delta_ts >= options_.ab_backtrack_ts)
        break;
    }

    std::cout << "...................................\n";
    std::cout << prv_odom_datas.size() << std::endl;
    std::cout << "...................................\n";

    Eigen::Matrix4d delta_pose = Eigen::Matrix4d::Identity();
    if(prv_odom_datas.size() > 1)
    {
      for(int i = 0; i < (int)prv_odom_datas.size() - 1; ++i)
      {
        printf("i: %d, %.9lf\n", i, prv_odom_datas[i].timestamp_);
        const loose_couple::OdomData prv_odom1 = prv_odom_datas[i];
        const loose_couple::OdomData prv_odom2 = prv_odom_datas[i + 1];
    
        Eigen::Matrix4d T_o1_o2 = Eigen::Matrix4d::Identity();
        T_o1_o2 = getDeltaPoseWithOdom(prv_odom1, prv_odom2, false);
        delta_pose *= T_o1_o2;
      }
    }


    std::cout << "###########################################\n";
    printf("%.9lf ---- %.9lf\n", timestamp, prv_odom_datas.back().timestamp_);
    std::cout << "delta_ts: " << timestamp - prv_odom_datas.back().timestamp_ << "\n";
    std::cout << "###########################################\n";

    if(timestamp > prv_odom_datas.back().timestamp_)
    {
      const loose_couple::OdomData prv_odom = prv_odom_datas.back();
      double delta_ts = timestamp - prv_odom_datas.back().timestamp_;

      Eigen::Vector3d delta_theta = prv_odom.angular_velocity_ * delta_ts;
      Eigen::Quaterniond delta_q(1.0f, delta_theta.x() * 0.5f, 
              delta_theta.y() * 0.5f, delta_theta.z() * 0.5f);
      Eigen::Vector3d delta_t = prv_odom.linear_velocity_ * delta_ts;

      Eigen::Matrix4d T_o1_o2 = Eigen::Matrix4d::Identity();
      T_o1_o2.block<3, 3>(0, 0) = delta_q.normalized().toRotationMatrix();
      T_o1_o2.block<3, 1>(0, 3) = delta_t;

      delta_pose *= T_o1_o2;
    }

    Eigen::Matrix4d delta_T = Tbo_ * delta_pose * Tob_;

    std::cout << "delta_T: \n" << delta_T << std::endl;

    if(options_.use_imu_rot)
    {
      loose_couple::ImuDatas imu_datas;
      const double &cur_ts = timestamp;
      const double &prv_ts = prv_odom_datas.front().timestamp_;
      align_ptr_->getImuMeasurements(prv_ts, cur_ts, imu_datas);
      if(!imu_datas.empty())
      {
        Eigen::Matrix3d delta_rot = getDeltaRotationWithImu(imu_datas);
        delta_T.block<3, 3>(0, 0) = delta_rot;
      }
    }

    global_pose_ = kf_T_w_b * delta_T;

    std::cout << "*********************************************\n";
    std::cout << "kf_T_w_b: \n" << kf_T_w_b << "\n\n";
    std::cout << "delta_T: \n" << delta_T << "\n\n";
    std::cout << "global_pose_: \n" << global_pose_ << "\n\n";
    std::cout << "*********************************************\n";
  }
  else
  {
    Eigen::Matrix4d latest_T_w_o = Eigen::Matrix4d::Identity();

    if(align_ptr_->matched_img_odoms_.count(timestamp))
    {
      const loose_couple::OdomData &latest_odom = align_ptr_->matched_img_odoms_[timestamp];
      latest_T_w_o.block<3, 3>(0, 0) = latest_odom.orientation_.toRotationMatrix();
      latest_T_w_o.block<3, 1>(0, 3) = latest_odom.position_;
    }
    else
    {
      double latest_img_ts = align_ptr_->matched_img_odom_ts_.back();
      const loose_couple::OdomData &latest_odom = align_ptr_->matched_img_odoms_[latest_img_ts];
      latest_T_w_o.block<3, 3>(0, 0) = latest_odom.orientation_.toRotationMatrix();
      latest_T_w_o.block<3, 1>(0, 3) = latest_odom.position_;

      double delta_ts = timestamp - latest_odom.timestamp_;

      Eigen::Vector3d delta_theta = latest_odom.angular_velocity_ * delta_ts;
      Eigen::Quaterniond delta_q(1.0f, delta_theta.x() * 0.5f, 
              delta_theta.y() * 0.5f, delta_theta.z() * 0.5f);
      Eigen::Vector3d delta_t = latest_odom.linear_velocity_ * delta_ts;

      Eigen::Matrix4d delta_pose = Eigen::Matrix4d::Identity();
      delta_pose.block<3, 3>(0, 0) = delta_q.normalized().toRotationMatrix();
      delta_pose.block<3, 1>(0, 3) = delta_t;

      latest_T_w_o = latest_T_w_o * delta_pose;
    }

    Eigen::Matrix4d kf_T_w_i = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d kf_T_w_o = Eigen::Matrix4d::Identity();
    for(int i = (int)global_data_.size() - 1; i >= 0; --i)
    {
      const loose_couple::VioData &global_data = global_data_[i];
      double prv_timestamp = global_data.timestamp_;

      if(!align_ptr_->matched_img_odoms_.count(prv_timestamp) && !global_data.is_keyframe_)
        continue;

      const loose_couple::OdomData &prv_odom = align_ptr_->matched_img_odoms_[prv_timestamp];
      kf_T_w_o.block<3, 3>(0, 0) = prv_odom.orientation_.toRotationMatrix();
      kf_T_w_o.block<3, 1>(0, 3) = prv_odom.position_;
      
      kf_T_w_i = global_data.T_w_b_;

      double delta_ts = timestamp - prv_timestamp;
      if(delta_ts >= options_.ab_backtrack_ts)
        break;
    }

    Eigen::Matrix4d delta_T = Tbo_ * inverse(kf_T_w_o) * latest_T_w_o * Tob_;

    global_pose_ = kf_T_w_i * delta_T;

    std::cout << "*********************************************\n";
    std::cout << "kf_T_w_i: \n" << kf_T_w_i << "\n\n";
    std::cout << "delta_T: \n" << delta_T << "\n\n";
    std::cout << "global_pose_: \n" << global_pose_ << "\n\n";
    std::cout << "*********************************************\n";
  }

  return global_pose_;
}

}
