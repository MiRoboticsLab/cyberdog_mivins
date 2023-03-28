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

#include "mivins/camera_models/equidistant_fisheye_geometry.h"

namespace vk
{
    namespace cameras
    {

        EquidistantFisheyeGeometry::EquidistantFisheyeGeometry(
            const int width, const int height, const double focal_length,
            const Eigen::Vector2d &principal_point, const double radius)
            : EquidistantFisheyeGeometry(width, height, EquidistantFisheyeProjection(focal_length, principal_point), radius)
        {
        }

        EquidistantFisheyeGeometry::EquidistantFisheyeGeometry(
            const int width, const int height,
            const EquidistantFisheyeProjection &projection, const double radius)
            : CameraGeometry<EquidistantFisheyeProjection>(width, height, projection)
        {
            cv::Mat mask = cv::Mat(imageHeight(), imageWidth(), CV_8UC1);
            for (uint32_t i = 0; i < imageHeight(); i++)
            {
                for (uint32_t j = 0; j < imageWidth(); j++)
                {
                    int64_t h_dist = static_cast<int64_t>(i) -
                                     static_cast<int64_t>(projection.principal_point()[1]);
                    int64_t w_dist = static_cast<int64_t>(j) -
                                     static_cast<int64_t>(projection.principal_point()[0]);
                    double dist_squared = std::pow(h_dist, 2) + std::pow(w_dist, 2);
                    if (dist_squared > radius * radius)
                        mask.at<uint8_t>(i, j) = 0;
                    else
                        mask.at<uint8_t>(i, j) = 1;
                }
            }
            setMask(mask);
        }

    } // namespace cameras
} // namespace vk
