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

#include "mivins/camera_models/no_distortion.h"
#include "mivins/camera_models/camera_geometry_base.h"

namespace vk
{
    namespace cameras
    {

        template <typename Distortion>
        class MeiProjection
        {
        public:
            typedef Distortion distortion_t;
            const CameraGeometryBase::Type cam_type_ = CameraGeometryBase::Type::kMei;

            MeiProjection() = default;

            MeiProjection(
                double fx, double fy, double cx, double cy, double xi, distortion_t distortion);

            // Intrinsic parameters ordering: fu, fv, cu, cv
            MeiProjection(const Eigen::VectorXd &intrinsics, distortion_t distortion);

            ~MeiProjection() = default;

            // Computes bearing vector from pixel coordinates. Z-component of the returned
            // bearing vector is 1. IMPORTANT: returned vector is NOT of unit length!
            bool backProject3(
                const Eigen::Ref<const Eigen::Vector2d> &keypoint,
                Eigen::Vector3d *out_point_3d) const;

            // Computes pixel coordinates from bearing vector.
            void project3(
                const Eigen::Ref<const Eigen::Vector3d> &point_3d,
                Eigen::Vector2d *out_keypoint,
                Eigen::Matrix<double, 2, 3> *out_jacobian_point) const;

            // Returns focal length (transforms unit plane error to pixel error).
            double errorMultiplier() const;

            double GetAngleError(double img_err) const;

            // Focal length will be width / 2, thus corresponding to a 90deg field of
            // view.
            static MeiProjection<Distortion> createTestProjection(
                const size_t image_width, const size_t image_height);

            template <typename T>
            const T *distortion() const;

            void print(std::ostream &out) const;

            enum IntrinsicParameters
            {
                kFocalLengthX,
                kFocalLengthY,
                kPricipalPointX,
                kPrincipalPointY,
                kXi
            };
            // Returns the intrinsic Parameters in vectof of form
            // [fx, fy, cx, cy, xi]
            Eigen::VectorXd getIntrinsicParameters() const;

            // returns the distortion Parameters of the underlying distortion model
            Eigen::VectorXd getDistortionParameters() const;

            double fx_ = 1;     // Focal length x.
            double fy_ = 1;     // Focal length y.
            double fx_inv_ = 1; // Inverse focal length x
            double fy_inv_ = 1; // Inverse focal length y
            double cx_ = 0;     // Principle point x.
            double cy_ = 0;     // Principle point y.
            double xi_ = 0;     // Xi.
            distortion_t distortion_;

            double inv_k11_;
            double inv_k13_;
            double inv_k22_;
            double inv_k23_;

            Eigen::Vector2d principal_point_;
            Eigen::DiagonalMatrix<double, 2> focal_matrix_;
        };

    } // namespace cameras
} // namespace vk

#include <mivins/camera_models/mei_projection.hpp>
