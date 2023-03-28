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

// geometry
/*
#include <mivins/camera_models/camera_geometry_base.h>
#include <mivins/camera_models/camera_geometry.h>
*/

// distortion models
/*
#include <mivins/camera_models/no_distortion.h>
#include <mivins/camera_models/atan_distortion.h>
#include <mivins/camera_models/equidistant_distortion.h>
#include <mivins/camera_models/radial_tangential_distortion.h>

// projections
#include <mivins/camera_models/pinhole_projection.h>
*/

namespace vk
{
    namespace cameras
    {

        template <typename ProjectionType>
        class CameraGeometry;

        template <typename DistrortionType>
        class PinholeProjection;

        template <typename DistrortionType>
        class MeiProjection;

        class AtanDistortion;
        class EquidistantDistortion;
        class NoDistortion;
        class RadialTangentialDistortion;

        typedef CameraGeometry<PinholeProjection<NoDistortion>> PinholeGeometry;
        typedef CameraGeometry<PinholeProjection<AtanDistortion>> PinholeAtanGeometry;
        typedef CameraGeometry<PinholeProjection<EquidistantDistortion>>
            PinholeEquidistantGeometry;
        typedef CameraGeometry<PinholeProjection<RadialTangentialDistortion>>
            PinholeRadTanGeometry;
        typedef CameraGeometry<MeiProjection<RadialTangentialDistortion>>
            MeiRadTanGeometry;

        class OmniGeometry;
    } // namespace cameras
} // namespace vk
