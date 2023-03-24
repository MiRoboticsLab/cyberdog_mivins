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

#include <map>
#include <vector>
#include <memory>

#include "initial/solve_5pts.h"
#include "initial/initial_sfm.h"
#include "initial/initial_alignment.h"
#include "initial/initial_ex_rotation.h"


#include "common/parameters.h"
#include "common/common_lib.h"
#include "estimator/frame_manager.h"

class Initializer
{
public:

  Initializer(std::vector<StateGroup> *states, Eigen::Vector3d *g,
              const FrameManagerPtr &frame_manager);

  ~Initializer();

  bool initialStructure(const int frame_count, 
      std::map<double, ImageFrame> &all_image_frame, 
      int &marginalization_flag);

  void setImuAvailable(bool use_imu);

  void setOdomAvailable(bool use_odom);

  bool visualInitialAlign(const int frame_count, 
    std::map<double, ImageFrame> &all_image_frame);

private:

  bool relativePose(Eigen::Matrix3d &relative_R, Eigen::Vector3d &relative_T, int &l);

  Eigen::Vector3d *g_;

  std::vector<StateGroup> *states_;

  FrameManagerPtr frame_manager_;

  MotionEstimator motion_estimator_;

  bool use_imu_;

  bool use_odom_;

  InitialAlignment::Ptr init_alignment_;
};

using InitializerPtr = std::shared_ptr<Initializer>;
