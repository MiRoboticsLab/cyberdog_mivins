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

#ifndef MIVINS_NODE_BASE_H_
#define MIVINS_NODE_BASE_H_
#include "./mivins_process.h"
#include <memory>
namespace mivins_ros2
{
class MivinsNodeBase: public rclcpp::Node
  {
public:
    MivinsNodeBase();
    ~MivinsNodeBase() = default;
private:
    mivins::PipelineType type_;
public:
    std::unique_ptr < mivins::MivinsProcess > mivins_process_;
  };
}  // namespace mivins_ros2
#endif  // MIVINS_NODE_BASE_H_
