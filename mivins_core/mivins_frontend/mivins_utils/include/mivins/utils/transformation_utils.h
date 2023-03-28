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

#ifndef TRANSFORMATION_UTILS_H_
#define TRANSFORMATION_UTILS_H_

#include <vector>

#include <mivins/utils/aligned.h>

#include <kindr/minimal/position.h>
#include <kindr/minimal/quat_transformation.h>
#include <kindr/minimal/rotation_quaternion.h>

namespace mivins
{

    typedef kindr::minimal::QuatTransformation Transformation;
    typedef Aligned<std::vector, Transformation> TransformationVector;
    typedef kindr::minimal::RotationQuaternion Quaternion;
    typedef kindr::minimal::AngleAxis AngleAxis;
    typedef kindr::minimal::Position Position3D;

    // Types used in ceres error terms, where templating for Jet types
    // is necessary.
    template <class Scalar>
    using QuaternionTemplate = kindr::minimal::RotationQuaternionTemplate<Scalar>;

    template <class Scalar>
    using PositionTemplate = kindr::minimal::PositionTemplate<Scalar>;

} // namespace mivins

#endif // TRANSFORMATION_UTILS_H_
