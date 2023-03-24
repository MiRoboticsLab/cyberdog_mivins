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

#ifndef MINKINDR_MINIMAL_COMMON_H
#define MINKINDR_MINIMAL_COMMON_H

#include <Eigen/Core>
#include <glog/logging.h>
//no use
namespace kindr
{
    namespace minimal
    {

        inline void SkewMatrix(const Eigen::Vector3d &v, Eigen::Matrix3d *skew)
        {
            CHECK_NOTNULL(skew);
            skew->setZero();
            (*skew)(0, 1) = -v[2];
            (*skew)(1, 0) = v[2];
            (*skew)(0, 2) = v[1];
            (*skew)(2, 0) = -v[1];
            (*skew)(1, 2) = -v[0];
            (*skew)(2, 1) = v[0];
        }

        inline void SkewMatrix(const Eigen::Vector3d &v, Eigen::Map<Eigen::Matrix3d> *skew)
        {
            CHECK_NOTNULL(skew);
            skew->setZero();
            (*skew)(0, 1) = -v[2];
            (*skew)(1, 0) = v[2];
            (*skew)(0, 2) = v[1];
            (*skew)(2, 0) = -v[1];
            (*skew)(1, 2) = -v[0];
            (*skew)(2, 1) = v[0];
        }

        inline Eigen::Matrix3d SkewMatrix(const Eigen::Vector3d &v)
        {
            Eigen::Matrix3d skew;
            SkewMatrix(v, &skew);
            return skew;
        }

    } // namespace minimal
} // namespace kindr

#endif // MINKINDR_MINIMAL_COMMON_H
