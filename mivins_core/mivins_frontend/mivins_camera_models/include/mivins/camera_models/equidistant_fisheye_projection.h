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

#include <Eigen/Core>

#include <mivins/camera_models/camera_geometry_base.h>

namespace vk
{
    namespace cameras
    {

        // May later be generalized to any distortion once a use case exists.
        class EquidistantFisheyeProjection
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            const CameraGeometryBase::Type cam_type_ = CameraGeometryBase::Type::kEqFisheye;
            EquidistantFisheyeProjection(
                double focal_length, Eigen::Vector2d principal_point);

            bool backProject3(
                const Eigen::Ref<const Eigen::Vector2d> &keypoint,
                Eigen::Vector3d *out_bearing_vector) const;

            void project3(
                const Eigen::Ref<const Eigen::Vector3d> &point_3d,
                Eigen::Vector2d *out_keypoint,
                Eigen::Matrix<double, 2, 3> *out_jacobian_point) const;

            double errorMultiplier() const;

            double GetAngleError(double img_err) const;

            void print(std::ostream &out) const;

            enum IntrinsicParameters
            {
                kFocalLength,
                kPrincipalPointX,
                kPrincipalPointY
            };
            // returns the intrinsic values as vector
            // [focal_length, cx, cy]
            Eigen::VectorXd getIntrinsicParameters() const;

            // Returns the distortion parameters
            Eigen::VectorXd getDistortionParameters() const;

            inline const Eigen::Vector2d &principal_point() const
            {
                return principal_point_;
            }

        private:
            const double focal_length_;
            const Eigen::Vector2d principal_point_;
        };

    } // namespace cameras
} // namespace vk
