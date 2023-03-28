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

#pragma once

#include "mivins/camera_models/camera_geometry.h"
#include "mivins/camera_models/equidistant_fisheye_projection.h"

namespace vk
{
    namespace cameras
    {

        // Equidistant fisheye geometry is strongly typed to enforce mask.
        class EquidistantFisheyeGeometry : public CameraGeometry<EquidistantFisheyeProjection>
        {
        public:
            EquidistantFisheyeGeometry(
                const int width, const int height, const double focal_length,
                const Eigen::Vector2d &principal_point, const double radius);

            EquidistantFisheyeGeometry(
                const int width, const int height,
                const EquidistantFisheyeProjection &projection, const double radius);

            virtual ~EquidistantFisheyeGeometry() = default;
        };

    } // namespace cameras
} // namespace vk
