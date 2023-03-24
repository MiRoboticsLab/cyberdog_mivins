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

#include <mivins/mivins_global_types.h>

namespace mivins
{

    struct StereoTriangulationOptions
    {
        size_t triangulate_n_features = 120;
        double mean_depth_inv = 1.0 / 3.0;
        double min_depth_inv = 1.0 / 1.0;
        double max_depth_inv = 1.0 / 50.0;
    };

    class StereoTriangulation
    {
    public:
        typedef std::shared_ptr<StereoTriangulation> Ptr;

        StereoTriangulationOptions options_;
        DetectorPtr feature_detector_;

        StereoTriangulation(
            const StereoTriangulationOptions &options,
            const DetectorPtr &feature_detector);
        ~StereoTriangulation() = default;

        void compute(const FramePtr &frame0, const FramePtr &frame1);
    };

} // namespace mivins
