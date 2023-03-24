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

#include <utility>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <mivins/common/types.h>
#include <mivins/tracker/feature_tracker_obs.h>

namespace mivins
{

    namespace feature_tracker_tools
    {

        /// pivot_ration needs to be in range(0,1) and if 0.5 it returns the median.
        double GetTracksDisparityPercentile(
            const FeatureTracksPerCam &tracks,
            double pivot_ratio);

        void GetFeatureMatches(
            const Frame &frame1, const Frame &frame2,
            std::vector<std::pair<size_t, size_t>> *matches_12);

        void GetFeatureMatches(
            const FramePtr &frame1, const FramePtr &frame2,
            std::unordered_map<size_t, size_t> &matches_12);

        void DrawFeatureMatches(
            const FramePtr &frame1, const FramePtr &frame2,
            std::unordered_map<size_t, size_t> &matches_12);

    } // namespace feature_tracker_tools
} // namespace mivins
