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

#ifndef FAST_DETECTION_GPU_H__
#define FAST_DETECTION_GPU_H__

#include "fast/fast.h"

#include <mivins/common/types.h>
#include <mivins/common/camera_fwd.h>
#include <mivins/common/occupancy_grid_2d.h>
#include <mivins/direct/feature_detector_types.h>

namespace mivins
{
    using ::std::vector;
    namespace feature_detector_utils
    {

        void FastDetectorGpu(
            const ImgPyramid &img_pyr,
            const int threshold,
            const int border,
            const size_t min_level,
            const size_t max_level,
            Corners &corners,
            OccupandyGrid2D& grid);
    
        void EdgeletDetectorV2Gpu(
            const ImgPyramid &img_pyr,
            const int threshold,
            const int border,
            const size_t min_level,
            const size_t max_level,
            Corners &corners,
            OccupandyGrid2D& grid);
    } // feature_detector_utils
} // namespace mivins

#endif