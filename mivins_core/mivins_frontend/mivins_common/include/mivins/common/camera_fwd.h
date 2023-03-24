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

#include <memory>

namespace vk
{
    namespace cameras
    {
        class CameraGeometryBase;
        class NCamera;
    }
}

namespace mivins
{
    using Camera = vk::cameras::CameraGeometryBase;
    using CameraPtr = std::shared_ptr<Camera>;
    using CameraConstPtr = std::shared_ptr<const Camera>;
    using CameraBundle = vk::cameras::NCamera;
    using CameraBundlePtr = std::shared_ptr<CameraBundle>;
} // namespace mivins
