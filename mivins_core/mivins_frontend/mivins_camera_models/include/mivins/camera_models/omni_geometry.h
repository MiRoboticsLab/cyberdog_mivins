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
#include "mivins/camera_models/omni_projection.h"

namespace vk
{
    namespace cameras
    {

        // Omni geometry is strongly typed to enforce mask.
        class OmniGeometry : public CameraGeometry<OmniProjection>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            constexpr static size_t kParamNum = 24;
            OmniGeometry(const int width, const int height,
                         const Eigen::Matrix<double, 5, 1> &polynomial,
                         const Eigen::Vector2d &principal_point,
                         const Eigen::Vector3d &distortion,
                         const Eigen::Matrix<double, 12, 1> &inverse_polynomial,
                         const Eigen::Vector2d &mask_relative_radii);
            // Version which does not apply a mask.
            OmniGeometry(const int width, const int height,
                         const Eigen::Matrix<double, 5, 1> &polynomial,
                         const Eigen::Vector2d &principal_point,
                         const Eigen::Vector3d &distortion,
                         const Eigen::Matrix<double, 12, 1> &inverse_polynomial);
            OmniGeometry(const int width, const int height,
                         const Eigen::VectorXd &intrinsics);

            virtual ~OmniGeometry() = default;

        private:
            static const Eigen::Vector2d kDontMask;
        };

    } // namespace cameras
} // namespace vk
