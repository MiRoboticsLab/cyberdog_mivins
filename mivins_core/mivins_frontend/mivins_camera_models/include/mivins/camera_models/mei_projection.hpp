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

#include <iostream>
#include <glog/logging.h>
#include <mivins/camera_models/mei_projection.h>

namespace vk
{
    namespace cameras
    {

        template <typename Distortion>
        MeiProjection<Distortion>::MeiProjection(
            double fx, double fy, double cx, double cy, double xi, distortion_t distortion)
            : fx_(fx), fy_(fy), fx_inv_(1.0 / fx_), fy_inv_(1.0 / fy_), cx_(cx), cy_(cy), xi_(xi), distortion_(distortion)
        {
        }

        template <typename Distortion>
        MeiProjection<Distortion>::MeiProjection(
            const Eigen::VectorXd &intrinsics, distortion_t distortion)
            : distortion_(distortion)
        {
            CHECK(intrinsics.size() == 5);
            fx_ = intrinsics(0);
            fy_ = intrinsics(1);
            cx_ = intrinsics(2);
            cy_ = intrinsics(3);
            xi_ = intrinsics(4);
            fx_inv_ = 1.0 / fx_;
            fy_inv_ = 1.0 / fy_;
        }

        template <typename Distortion>
        bool MeiProjection<Distortion>::backProject3(
            const Eigen::Ref<const Eigen::Vector2d> &keypoint,
            Eigen::Vector3d *out_bearing_vector) const
        {
            double lambda;

            double x = (keypoint[0] - cx_) * fx_inv_;
            double y = (keypoint[1] - cy_) * fy_inv_;
            distortion_.undistort(x, y);
            double mx_u = x;
            double my_u = y;
            double xi = xi_;
            if (xi == 1.0)
            {
                lambda = 2.0 / (mx_u * mx_u + my_u * my_u + 1.0);
                (*out_bearing_vector)[0] = lambda * mx_u;
                (*out_bearing_vector)[1] = lambda * my_u;
                (*out_bearing_vector)[2] = lambda - 1.0;
            }
            else
            {
                lambda = (xi + sqrt(1.0 + (1.0 - xi * xi) * (mx_u * mx_u + my_u * my_u))) / (1.0 + mx_u * mx_u + my_u * my_u);
                (*out_bearing_vector)[0] = lambda * mx_u;
                (*out_bearing_vector)[1] = lambda * my_u;
                (*out_bearing_vector)[2] = lambda - xi;
            }
            out_bearing_vector->normalize();
            return true;
        }

        template <typename Distortion>
        void MeiProjection<Distortion>::project3(
            const Eigen::Ref<const Eigen::Vector3d> &point_3d,
            Eigen::Vector2d *out_keypoint,
            Eigen::Matrix<double, 2, 3> *out_jacobian_point) const
        {
            const double z_inv = 1 / (point_3d(2) + xi_ * point_3d.norm());
            // TODO(tcies) precalced member?
            const Eigen::DiagonalMatrix<double, 2> focal_matrix(fx_, fy_);
            const Eigen::Vector2d uv = point_3d.head<2>() * z_inv;
            (*out_keypoint) =
                focal_matrix * distortion_.distort(uv) + Eigen::Vector2d(cx_, cy_);

            // WZJ::TODO::
            if (out_jacobian_point)
            {

                Eigen::Matrix<double, 2, 3> duv_dxy;
                duv_dxy(0, 0) = z_inv - xi_ * point_3d(0) * point_3d(0) * z_inv * z_inv / point_3d.norm();
                duv_dxy(0, 1) = (-1.0) * xi_ * point_3d(0) * point_3d(1) * z_inv * z_inv / point_3d.norm();
                duv_dxy(0, 2) = (-1.0) * point_3d(0) * z_inv * z_inv * (1.0 + xi_ * point_3d(2) / point_3d.norm());
                duv_dxy(1, 0) = (-1.0) * xi_ * point_3d(0) * point_3d(1) * z_inv * z_inv / point_3d.norm();
                duv_dxy(1, 1) = z_inv - xi_ * point_3d(1) * point_3d(1) * z_inv * z_inv / point_3d.norm();
                duv_dxy(1, 2) = (-1.0) * point_3d(1) * z_inv * z_inv * (1.0 + xi_ * point_3d(2) / point_3d.norm());
                //duv_dxy.leftCols<2>() = Eigen::Matrix2d::Identity() * z_inv;
                //duv_dxy.rightCols<1>() = - point_3d.head<2>() * z_inv * z_inv;

                (*out_jacobian_point) = focal_matrix * distortion_.jacobian(uv) * duv_dxy;
            }
        }

        template <typename Distortion>
        double MeiProjection<Distortion>::errorMultiplier() const
        {
            return 1.0;
        }

        template <typename Distortion>
        double MeiProjection<Distortion>::GetAngleError(double img_err) const
        {
            Eigen::Vector3d center_f;
            backProject3(Eigen::Vector2d(cx_, cy_), &center_f);
            const Eigen::Vector2d offset_x(
                cx_ + img_err, cy_);
            Eigen::Vector3d offset_x_f;
            backProject3(offset_x, &offset_x_f);
            const Eigen::Vector2d offset_y(
                cx_, cy_ + img_err);
            Eigen::Vector3d offset_y_f;
            backProject3(offset_y, &offset_y_f);

            const double theta_x = std::acos(center_f.dot(offset_x_f));
            const double theta_y = std::acos(center_f.dot(offset_y_f));

            return (theta_x + theta_y) / 2.0;
        }

        template <typename Distortion>
        MeiProjection<Distortion>
        MeiProjection<Distortion>::createTestProjection(
            const size_t image_width, const size_t image_height)
        {
            return MeiProjection(
                image_width / 2, image_width / 2, image_width / 2, image_height / 2, 1.0 /*Xi*/,
                Distortion::createTestDistortion());
        }

        template <typename Distortion>
        template <typename T>
        const T *MeiProjection<Distortion>::distortion() const
        {
            return dynamic_cast<const T *>(&distortion_);
        }

        template <typename Distortion>
        void MeiProjection<Distortion>::print(std::ostream &out) const
        {
            out << "  Projection = Mei" << std::endl;
            out << "  Focal length = (" << fx_ << ", " << fy_ << ")" << std::endl;
            out << "  Principal point = (" << cx_ << ", " << cy_ << ")" << std::endl;
            out << "  Xi = (" << xi_ << ")" << std::endl;
            distortion_.print(out);
        }

        template <typename Distortion>
        Eigen::VectorXd MeiProjection<Distortion>::getIntrinsicParameters() const
        {
            Eigen::VectorXd intrinsics(5);
            intrinsics(0) = fx_;
            intrinsics(1) = fy_;
            intrinsics(2) = cx_;
            intrinsics(3) = cy_;
            intrinsics(4) = xi_;
            return intrinsics;
        }

        template <typename Distortion>
        Eigen::VectorXd MeiProjection<Distortion>::getDistortionParameters() const
        {
            return distortion_.getDistortionParameters();
        }

    } // namespace cameras
} // namespace vk
