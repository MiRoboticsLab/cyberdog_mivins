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

#include <numeric>

// included in global.h
// #include <svo/common/frame.h>
// #include <svo/common/types.h>
// #include <svo/imu_handler.h>
// #include <svo/odom_handler.h>

#include "common/global.h"
#include "common/parameters.h"

class VinsMotionDetector
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<VinsMotionDetector> Ptr;

  VinsMotionDetector(const VinsMotionDetectorOptions &options): opt_(options){}
  ~VinsMotionDetector(){}

  const VinsMotionDetectorOptions opt_;

  bool isImageMoving(double &sigma) const;

  bool isImuMoving(const int imu_rate, const ImuMeasurements &imu_measurements);

  bool isOdomMoving(const int odom_rate, const OdomMeasurements &odom_measurements);

  void setFrames(const FrameBundlePtr &last_frames,
                 const FrameBundlePtr &new_frames)
  {
    last_frames_= last_frames;
    new_frames_ = new_frames;
  }

  inline bool lastCheckValid() const
  {
    return last_check_valid_;
  }

private:
  FrameBundlePtr last_frames_;
  FrameBundlePtr new_frames_;

  int findFeatureCorrespondences(Keypoints &features_last,
                                 Keypoints &features_new) const;

  bool findFeatureCorrespondence(const size_t cam_index,
                                 const int track_id,
                                 Keypoint *last_px) const;

  double stdVec(const std::vector<double>& v);
  
  mutable bool last_check_valid_ = false;
};