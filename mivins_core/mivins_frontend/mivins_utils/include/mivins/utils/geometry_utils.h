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

#ifndef GEOMETRY_UTILS_H_
#define GEOMETRY_UTILS_H_

#include <vector>
#include <Eigen/Core>

namespace vk
{

    using BearingVector = Eigen::Vector3d;
    using Bearings = Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::ColMajor>;

    struct Homography
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3d t_cur_ref;
        Eigen::Matrix3d R_cur_ref;
        Eigen::Vector3d n_cur;
        double score;
        Homography()
            : t_cur_ref(), R_cur_ref(), n_cur(), score(0.0)
        {
        }
    };

    /// Estimates Homography from corresponding feature bearing vectors.
    /// Score of returned homography is set to the number of inliers.
    Homography estimateHomography(
        const Bearings &f_cur,
        const Bearings &f_ref,
        const double focal_length,
        const double reproj_error_thresh,
        const size_t min_num_inliers);

} // namespace vk

#endif // VIKIT_HOMOGRAPHY_H_
