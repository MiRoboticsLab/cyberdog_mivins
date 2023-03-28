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

#include "estimator/motion_detector.h"

bool VinsMotionDetector::isImageMoving(double &sigma) const
{
  last_check_valid_ = false;
  if(!last_frames_)
  {
    return true;
  }

  //pre-check if there are enough features
  if( numFeatures(last_frames_) < size_t(opt_.min_number_correspondences)
     || numFeatures(new_frames_) < size_t(opt_.min_number_correspondences))
  {
    return true;
  }

  // find correspondences between last and new frame
  Keypoints features_last(2, opt_.max_features_to_check);
  Keypoints features_new(2, opt_.max_features_to_check);
  int num_correspondences = findFeatureCorrespondences(features_last,
                                                               features_new);

  if(num_correspondences < opt_.min_number_correspondences)
  {
    return true;
  }

  //find the number of moving pixels
  last_check_valid_ = true;
  const int n_moving_pixels_threshold =
      static_cast<int>(opt_.ratio_moving_pixels_threshold*num_correspondences);
  int n_moving_pixels = 0;
  for(int i = 0; i < num_correspondences; ++i)
  {
    //compute pixel distance
    if((features_last.col(i)-features_new.col(i)).norm()
       > opt_.px_diff_threshold)
    {
      ++n_moving_pixels;
      if(n_moving_pixels > n_moving_pixels_threshold)
      {
        return true;
      }
    }
  }
  //! @todo more sophisticated assignment of sigma
  sigma = opt_.sigma;
  return false;
}

bool VinsMotionDetector::isImuMoving(const int imu_rate, 
            const ImuMeasurements &imu_measurements)
{
  if(imu_measurements.empty())
    return true;
  
  const size_t imu_size = imu_measurements.size();
  const double sqrt_dt = std::sqrt(1.0 / imu_rate);
  std::vector<double> gyr_x (imu_size);
  std::vector<double> gyr_y (imu_size);
  std::vector<double> gyr_z (imu_size);
  std::vector<double> acc_x (imu_size);
  std::vector<double> acc_y (imu_size);
  std::vector<double> acc_z (imu_size);

  // for (int i = imu_measurements.size() - 1; i >= 0; --i)
  for(size_t i = 0; i < imu_size; ++i)
  {
    double ts = imu_measurements[i].timestamp_;
    const ImuMeasurement& m = imu_measurements[i];
    
    gyr_x[i] = m.angular_velocity_.x();
    gyr_y[i] = m.angular_velocity_.y();
    gyr_z[i] = m.angular_velocity_.z();
    acc_x[i] = m.linear_acceleration_.x();
    acc_y[i] = m.linear_acceleration_.y();
    acc_z[i] = m.linear_acceleration_.z();
  }

  std::array<double, 3> gyr_std {0.0, 0.0, 0.0};
  std::array<double, 3> acc_std {0.0, 0.0, 0.0};
  gyr_std[0] = stdVec(gyr_x) * sqrt_dt;
  gyr_std[1] = stdVec(gyr_y) * sqrt_dt;
  gyr_std[2] = stdVec(gyr_z) * sqrt_dt;
  acc_std[0] = stdVec(acc_x) * sqrt_dt;
  acc_std[1] = stdVec(acc_y) * sqrt_dt;
  acc_std[2] = stdVec(acc_z) * sqrt_dt;

  bool stationary = true;
  for (size_t idx = 0; idx < 3; idx ++)
  {
    stationary &= (gyr_std[idx] < 6e-5);
    stationary &= (acc_std[idx] < 10e-4);
  }

  return !stationary;
}

bool VinsMotionDetector::isOdomMoving(const int odom_rate, 
        const OdomMeasurements &odom_measurements)
{
  if(odom_measurements.empty())
    return true;
  // TODO

  return true;
}

int VinsMotionDetector::findFeatureCorrespondences(
    Keypoints &features_last, Keypoints &features_new) const
{
  int num_correspondences = 0;
  Keypoint px;
  for(const FramePtr &frame : *new_frames_)
  {
    size_t cam_index = static_cast<size_t>(getNFrameIndex(frame));
    const int num_features = static_cast<int>(numFeatures(frame));
    for(int i = 0; i<num_features; ++i)
    {
      //check if track ID is assigned (seed or landmark)
      if(frame->track_id_vec_[i] == -1)
      {
        continue;
      }

      if(findFeatureCorrespondence(cam_index, frame->track_id_vec_[i], &px))
      {
        features_last.col(num_correspondences) = px;
        features_new.col(num_correspondences) = frame->px_vec_.col(i);
        ++num_correspondences;
        if(num_correspondences == opt_.max_features_to_check)
        {
          return num_correspondences;
        }
      }
    }
  }
  return num_correspondences;
}

bool VinsMotionDetector::findFeatureCorrespondence(
  const size_t cam_index, const int track_id, Keypoint *last_px) const
{
  const FramePtr &last_frame = last_frames_->at(cam_index);
  //find the matching track ID in last frames
  const int num_features =
      static_cast<int>(numFeatures(last_frame));
  for(int i = 0; i<num_features; ++i)
  {
    if(last_frame->track_id_vec_[i] == track_id)
    {
      *last_px = last_frame->px_vec_.col(i);
      return true;
    }
  }
  return false;
}

double VinsMotionDetector::stdVec(const std::vector<double>& v)
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
