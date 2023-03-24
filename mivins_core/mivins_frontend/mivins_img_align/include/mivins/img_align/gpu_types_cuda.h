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

#pragma once

#include <imp/core/pixel.hpp>

namespace mivins
{

#define USE_SINGLE_PRECISION

#ifdef USE_SINGLE_PRECISION
    typedef float FloatTypeGpu;
#else
    typedef double FloatTypeGpu;
#endif

#ifdef USE_SINGLE_PRECISION
    typedef imp::Pixel32fC1 FloatPixelGpu;
    typedef float2 Float2TypeGpu;
    typedef imp::Pixel32fC2 Float2PixelGpu;
    typedef float3 Float3TypeGpu;
    typedef imp::Pixel32fC3 Float3PixelGpu;
    typedef std::uint32_t UIntTypeGpu;
    typedef imp::Pixel32uC1 UIntPixelGpu;
    typedef unsigned char BoolTypeGpu;
    typedef imp::Pixel8uC1 BoolPixelGpu;
#else
    // use double precision
    // TODO: Define Pixel64 in imp
    typedef imp::Pixel64fC1 FloatPixelGpu;
    typedef double2 Float2TypeGpu;
    typedef imp::Pixel64fC2 Float2PixelGpu;
    typedef double3 Float3TypeGpu;
    typedef imp::Pixel64fC3 Float3PixelGpu;
    typedef std::uint32_t UIntTypeGpu;
    typedef imp::Pixel32uC1 UIntPixelGpu;
    typedef unsigned char BoolTypeGpu;
    typedef imp::Pixel8uC1 BoolPixelGpu;
#endif

} // namespace mivins
