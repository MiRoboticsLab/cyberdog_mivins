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

#include "include/mivins_node_base.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

namespace mivins_ros2
{
MivinsNodeBase::MivinsNodeBase()
: Node("mivins_node")
{
  mivins_process_.reset(new mivins::MivinsProcess(this));
  if (MiVins_GetImuStatus()) {
    mivins_process_->subscribeImu();
  }

  if (MiVins_GetOdomStatus()) {
    mivins_process_->subscribeOdom();
  }

  mivins_process_->subscribeImage();
  mivins_process_->subscribeRemoteKey();

  mivins_process_->subscribeReloc();
  mivins_process_->subscribeMapServer();
}
}  // namespace mivins_ros2
