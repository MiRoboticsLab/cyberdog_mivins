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

#include <string>
#include <memory>
#include <Eigen/Core>
#include <mivins/camera_models/camera_geometry_base.h>

namespace vk
{
    namespace cameras
    {

        template <typename Projection>
        class CameraGeometry : public CameraGeometryBase
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            typedef Projection projection_t;

            CameraGeometry(
                const int width,
                const int height,
                const projection_t &projection);

            virtual ~CameraGeometry() = default;

            virtual bool backProject3(
                const Eigen::Ref<const Eigen::Vector2d> &keypoint,
                Eigen::Vector3d *out_point_3d) const override;

            virtual const ProjectionResult project3(
                const Eigen::Ref<const Eigen::Vector3d> &point_3d,
                Eigen::Vector2d *out_keypoint,
                Eigen::Matrix<double, 2, 3> *out_jacobian_point) const override;

            virtual void printParameters(std::ostream &out, const std::string &s = "Camera: ") const override;

            virtual Eigen::VectorXd getIntrinsicParameters() const override;

            virtual Eigen::VectorXd getDistortionParameters() const override;

            virtual double errorMultiplier() const override;

            virtual double GetAngleError(double img_err) const override;

            static std::shared_ptr<CameraGeometry<Projection>> createTestCamera();

            template <typename T = Projection>
            const T *projection() const;

        private:
            projection_t projection_;
        };

    } // namespace cameras
} // namespace vk

#include <mivins/camera_models/camera_geometry.hpp>
