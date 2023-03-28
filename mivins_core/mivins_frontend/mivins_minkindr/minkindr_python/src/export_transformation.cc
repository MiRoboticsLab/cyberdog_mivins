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

#include <kindr/minimal/quat_transformation.h>
#include <numpy_eigen/boost_python_headers.hpp>

using namespace boost::python;

using Transformation = kindr::minimal::QuatTransformationTemplate<double>;

Eigen::Vector3d GetPosition(const Transformation &transformation)
{
    return transformation.GetPosition();
}

Transformation::Rotation GetRotation(const Transformation &transformation)
{
    return transformation.GetRotation();
}

void ExportTransformation()
{
    using namespace boost::python;

    class_<Transformation, boost::shared_ptr<Transformation>>("Transformation", init<>())
        .def(init<const Eigen::Matrix4d &>())
        .def(init<const Transformation::Rotation &, const Transformation::Position &>())
        .def("getTransformationMatrix", &Transformation::getTransformationMatrix)
        .def("getRotation", getRotation)
        .def("getRotationMatrix", &Transformation::getRotationMatrix)
        .def("getPosition", getPosition)
        .def("inverse", &Transformation::inverse)
        .def(self * self);

    def("interpolateLinearly", &kindr::minimal::interpolateComponentwise<double>);
}
