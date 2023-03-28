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

#include <Eigen/Dense>

#include <mivins/camera_models/camera_geometry_base.h>

namespace vk
{
    namespace cameras
    {

        class OmniProjection
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            static constexpr int kInversePolynomialOrder = 12;
            const CameraGeometryBase::Type cam_type_ = CameraGeometryBase::Type::kOmni;

            // TODO(tcies) Outsource distortion to distortion class?
            OmniProjection(const Eigen::Matrix<double, 5, 1> &polynomial,
                           const Eigen::Vector2d &principal_point,
                           const Eigen::Vector3d &distortion,
                           const Eigen::Matrix<double, kInversePolynomialOrder, 1> &
                               inverse_polynomial);

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
                kPolinomial0,
                kPolinomial1,
                kPolinomial2,
                kPolinomial3,
                kPolinomial4,
                kPrincipalPointX,
                kPrincipalPointY
            };
            // Returns the parameters in Vector form:
            // [polynomial(0) ... polynomial(4), cx_, cy_]
            Eigen::VectorXd getIntrinsicParameters() const;

            // Returns the distortion parameters
            Eigen::VectorXd getDistortionParameters() const;

        private:
            const Eigen::Matrix<double, 5, 1> polynomial_;
            const Eigen::Vector2d principal_point_;
            const Eigen::Matrix<double, 12, 1> inverse_polynomial_;

            const Eigen::Matrix2d affine_correction_;
            const Eigen::Matrix2d affine_correction_inverse_;
        };

    } // namespace cameras
} // namespace vk
