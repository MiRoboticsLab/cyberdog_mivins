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

#include <mivins/common/types.h>

namespace mivins
{

    //------------------------------------------------------------------------------
    /// Temporary container used for corner detection. Features are initialized from these.
    typedef struct Corner
    {
        int x;       ///< x-coordinate of corner in the image.
        int y;       ///< y-coordinate of corner in the image.
        int level;   ///< pyramid level of the corner.
        float score; ///< shi-tomasi score of the corner.
        float angle; ///< for gradient-features: dominant gradient angle.

        Corner(int _x, int _y, float _score, int _level, float _angle)
            : x(_x), y(_y), level(_level), score(_score), angle(_angle)
        {
        }
    } Corner;
    using Corners = std::vector<Corner>;

    //------------------------------------------------------------------------------
    /// Available Feature Detector Types
    enum class DetectorType
    {
        kFast,             ///< Fast Corner Detector by Edward Rosten
        kGrad,             ///< Gradient detector for edgelets
        kFastGrad,         ///< Combined Fast and Gradient detector
        kShiTomasi,        ///< Shi-Tomasi detector
        kShiTomasiGrad,    ///< Combined Shi-Tomasi, fast and Gradient detector
        kGridGrad,         ///< Gradient detector with feature grid.
        kAll,              ///< Every pixel is a feature!
        kGradHuangMumford, ///< 'Natural' edges (see Huang CVPR'99)
        kCanny,            ///< Canny edge detector
        kSobel,            ///< Sobel edge detector
        kSobelGrad
    };

    //------------------------------------------------------------------------------
    /// Common options of all feature detectors.
    typedef struct DetectorOptions
    {
        /// Maximum one feature per bucked with cell_size width and height
        size_t cell_size = 30;

        /// Extract features on pyramid
        int max_level = 2;

        /// minimum pyramid level at which features should be selected
        int min_level = 0;

        /// no feature should be within border.
        int border = 8;

        /// Choose between {FAST, FAST_GRAD}, FAST_GRAD will use Edgelets.
        DetectorType detector_type = DetectorType::kFast;

        /// Primary detector threshold
        double threshold_primary = 10.0;

        /// Secondary detector threshold.
        /// Used if the detector uses two different detectors. E.g. in the case of
        /// FAST_GRAD, it is the gradient detector threshold.
        double threshold_secondary = 100.0;

        /// Level where features are initialized.
        /// Only for detectors supporting specific feature levels like the AllPixelDetector.
        int sampling_level = 0;
        int level = 0;

        /// fineness level of the secondary grid (used for extra shi tomasi features when loop closing is enabled)
        size_t sec_grid_fineness = 1;

        /// Corner Strength Thrshold for shitomasi features (used only when loop closing is enabled)
        double threshold_shitomasi = 100.0;
    } DetectorOptions;

} // namespace mivins
