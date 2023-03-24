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

#include <glog/logging.h>

namespace vk
{
    namespace cameras
    {

        template <typename DerivedKeyPoint>
        bool CameraGeometryBase::isKeypointVisible(
            const Eigen::MatrixBase<DerivedKeyPoint> &keypoint) const
        {
            EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DerivedKeyPoint, 2, 1);
            typedef typename DerivedKeyPoint::Scalar Scalar;
            if (camera_type_ == CameraGeometryBase::Type::kMei)
            {
                int64_t h_dist = static_cast<int64_t>(keypoint[1]) -
                                 static_cast<int64_t>(imageHeight() / 2);
                int64_t w_dist = static_cast<int64_t>(keypoint[0]) -
                                 static_cast<int64_t>(imageWidth() / 2);
                double dist_squared = std::pow(h_dist, 2) + std::pow(w_dist, 2);
                return (dist_squared < static_cast<Scalar>((imageWidth() - 0) * (imageWidth() - 0) / 4) && keypoint[0] >= static_cast<Scalar>(0.0) && keypoint[1] >= static_cast<Scalar>(0.0) && keypoint[0] < static_cast<Scalar>(imageWidth()) && keypoint[1] < static_cast<Scalar>(imageHeight())); //imageWidth()* imageWidth()/4)
            }
            else
            {
                return keypoint[0] >= static_cast<Scalar>(0.0) && keypoint[1] >= static_cast<Scalar>(0.0) && keypoint[0] < static_cast<Scalar>(imageWidth()) && keypoint[1] < static_cast<Scalar>(imageHeight());
            }
        }

        template <typename DerivedKeyPoint>
        bool CameraGeometryBase::isKeypointVisibleWithMargin(
            const Eigen::MatrixBase<DerivedKeyPoint> &keypoint,
            typename DerivedKeyPoint::Scalar margin) const
        {
            typedef typename DerivedKeyPoint::Scalar Scalar;
            EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DerivedKeyPoint, 2, 1);
            CHECK_LT(2 * margin, static_cast<Scalar>(imageWidth()));
            CHECK_LT(2 * margin, static_cast<Scalar>(imageHeight()));
            return keypoint[0] >= margin && keypoint[1] >= margin && keypoint[0] < (static_cast<Scalar>(imageWidth()) - margin) && keypoint[1] < (static_cast<Scalar>(imageHeight()) - margin);
        }

    } // namespace cameras
} // namespace vk
