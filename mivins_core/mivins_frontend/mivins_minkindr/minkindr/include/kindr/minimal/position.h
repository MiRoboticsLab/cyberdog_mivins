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

#ifndef KINDR_MINIMAL_POSITION_H
#define KINDR_MINIMAL_POSITION_H

#include <Eigen/Dense>
//Transformation
namespace kindr
{
    namespace minimal
    {

        template <typename Scalar>
        using PositionTemplate = Eigen::Matrix<Scalar, 3, 1>;

        typedef PositionTemplate<double> Position;

    } // namespace minimal
} // namespace kindr

#endif /* KINDR_MINIMAL_POSITION_H */