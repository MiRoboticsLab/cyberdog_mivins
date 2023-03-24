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

#include <glog/logging.h>
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <yaml-cpp/yaml.h>
#pragma diagnostic pop

namespace vk
{
    namespace cameras
    {
        class CameraGeometryBase;
    } // namespace cameras
} // namespace vk

namespace YAML
{

    template <>
    struct convert<std::shared_ptr<vk::cameras::CameraGeometryBase>>
    {
        /// This function will attempt to parse a camera from the yaml node.
        /// By default, yaml-cpp will throw and exception if the parsing fails.
        /// This function was written to *not* throw exceptions. Hence, decode always
        /// returns true, but when it fails, the shared pointer will be null.
        static bool decode(const Node &node, std::shared_ptr<vk::cameras::CameraGeometryBase> &camera);
        static Node encode(const std::shared_ptr<vk::cameras::CameraGeometryBase> &camera);
    };

    template <>
    struct convert<vk::cameras::CameraGeometryBase>
    {
        static bool decode(const Node &node, vk::cameras::CameraGeometryBase &camera);
        static Node encode(const vk::cameras::CameraGeometryBase &camera);
    };

} // namespace YAML
