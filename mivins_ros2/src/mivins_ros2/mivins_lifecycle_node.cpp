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

#include <memory>
#include "include/mivins_lifecycle.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  LOGGER_MAIN_INSTANCE("vins");
  std::shared_ptr<mivins_ros2::MIVINSLIFECYCLE> node_handle_ =
    std::make_shared<mivins_ros2::MIVINSLIFECYCLE>();
  rclcpp::spin(node_handle_->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
