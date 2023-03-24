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
#include <mivins/camera_models/camera_geometry.h>

namespace vk
{
    namespace cameras
    {

        template <typename Projection>
        CameraGeometry<Projection>::CameraGeometry(
            const int width,
            const int height,
            const projection_t &projection)
            : CameraGeometryBase(width, height), projection_(projection)
        {
            // NOTE: declare and specify this type within projection class
            camera_type_ = projection_.cam_type_;
            if (camera_type_ == Type::kMei)
            {
                cv::Mat mask = cv::Mat(imageHeight(), imageWidth(), CV_8UC1);
                for (uint32_t i = 0; i < imageHeight(); i++)
                {
                    for (uint32_t j = 0; j < imageWidth(); j++)
                    {
                        int64_t h_dist = static_cast<int64_t>(i) -
                                         static_cast<int64_t>(imageHeight() / 2);
                        int64_t w_dist = static_cast<int64_t>(j) -
                                         static_cast<int64_t>(imageWidth() / 2);
                        double dist_squared = std::pow(h_dist, 2) + std::pow(w_dist, 2);
                        if (dist_squared > (imageWidth() - 0) * (imageWidth() - 0) / 4)
                            mask.at<uint8_t>(i, j) = 0;
                        else
                            mask.at<uint8_t>(i, j) = 1;
                    }
                }
                setMask(mask);
            }
        }

        template <typename Projection>
        bool CameraGeometry<Projection>::backProject3(
            const Eigen::Ref<const Eigen::Vector2d> &keypoint,
            Eigen::Vector3d *out_point_3d) const
        {
            CHECK_NOTNULL(out_point_3d);
            return projection_.backProject3(keypoint, out_point_3d);
        }

        template <typename Projection>
        const ProjectionResult CameraGeometry<Projection>::project3(
            const Eigen::Ref<const Eigen::Vector3d> &point_3d,
            Eigen::Vector2d *out_keypoint,
            Eigen::Matrix<double, 2, 3> *out_jacobian_point) const
        {
            projection_.project3(point_3d, out_keypoint, out_jacobian_point);
            if (!isKeypointVisible(*out_keypoint))
                return ProjectionResult::Status::KEYPOINT_OUTSIDE_IMAGE_BOX;
            return ProjectionResult::Status::KEYPOINT_VISIBLE;
        }

        template <typename Projection>
        void CameraGeometry<Projection>::printParameters(
            std::ostream &out, const std::string &s) const
        {
            out << s << std::endl
                << "  name = " << label_ << std::endl
                << "  size = [" << width_ << ", " << height_ << "]" << std::endl;
            projection_.print(out);
        }

        template <typename Projection>
        Eigen::VectorXd CameraGeometry<Projection>::getIntrinsicParameters() const
        {
            return projection_.getIntrinsicParameters();
        }

        template <typename Projection>
        Eigen::VectorXd CameraGeometry<Projection>::getDistortionParameters() const
        {
            return projection_.getDistortionParameters();
        }

        template <typename Projection>
        double CameraGeometry<Projection>::errorMultiplier() const
        {
            return projection_.errorMultiplier();
        }

        template <typename Projection>
        double CameraGeometry<Projection>::GetAngleError(double img_err) const
        {
            return projection_.GetAngleError(img_err);
        }

        template <typename Projection>
        std::shared_ptr<CameraGeometry<Projection>>
        CameraGeometry<Projection>::createTestCamera()
        {
            const size_t width = 640;
            const size_t height = 480;
            return std::make_shared<CameraGeometry<Projection>>(
                width, height, Projection::createTestProjection(width, height));
        }

        template <typename Projection>
        template <typename T>
        const T *CameraGeometry<Projection>::projection() const
        {
            return dynamic_cast<const T *>(&projection_);
        }

    } // namespace cameras
} // namespace vk
