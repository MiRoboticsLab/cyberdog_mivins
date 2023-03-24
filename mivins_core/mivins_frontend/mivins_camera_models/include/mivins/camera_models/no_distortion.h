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

#include <iostream>

#include <Eigen/Dense>

namespace vk
{
    namespace cameras
    {

        class NoDistortion
        {
        public:
            NoDistortion() = default;
            ~NoDistortion() = default;

            inline void distort(
                double & /*u*/, double & /*v*/) const
            {
            }

            inline Eigen::Vector2d distort(const Eigen::Vector2d &vector) const
            {
                return vector;
            }

            inline Eigen::Matrix2d jacobian(const Eigen::Vector2d & /*vector*/) const
            {
                return Eigen::Matrix2d::Identity();
            }

            inline void undistort(
                double & /*u*/, double & /*v*/) const
            {
            }

            inline void print(std::ostream &out) const
            {
                out << "  Distortion: No" << std::endl;
            }

            // returns distortion parameters as vector (here empty vector)
            inline Eigen::VectorXd getDistortionParameters() const
            {
                Eigen::VectorXd distortion;
                return distortion;
            }

            static inline NoDistortion createTestDistortion()
            {
                return NoDistortion();
            }
        };

    } // namespace cameras
} // namespace vk
