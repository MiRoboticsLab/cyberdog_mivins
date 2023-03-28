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

#include <memory>
#include <mivins/common/types.h>

namespace mivins
{

    // forward declaration
    class Point;
    using PointPtr = std::shared_ptr<Point>;

    class Frame;
    using FramePtr = std::shared_ptr<Frame>;

    struct SeedRef
    {
        FramePtr keyframe;
        int seed_id = -1;
        SeedRef(const FramePtr &_keyframe, const int _seed_id)
            : keyframe(_keyframe), seed_id(_seed_id)
        {
            ;
        }
        SeedRef() = default;
        ~SeedRef() = default;
    };

    /** @todo (MWE)
 */
    struct FeatureWrapper
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        FeatureType &type;               //!< Type can be corner or edgelet.
        Eigen::Ref<Keypoint> px;         //!< Coordinates in pixels on pyramid level 0.
        Eigen::Ref<BearingVector> f;     //!< Unit-bearing vector of the feature.
        Eigen::Ref<GradientVector> grad; //!< Dominant gradient direction for edglets, normalized.
        Score &score;
        Level &level; //!< Image pyramid level where feature was extracted.
        PointPtr &landmark;
        SeedRef &seed_ref;
        int &track_id;

        FeatureWrapper(
            FeatureType &_type,
            Eigen::Ref<Keypoint> _px,
            Eigen::Ref<BearingVector> _f,
            Eigen::Ref<GradientVector> _grad,
            Score &_score,
            Level &_pyramid_level,
            PointPtr &_landmark,
            SeedRef &_seed_ref,
            int &_track_id)
            : type(_type), px(_px), f(_f), grad(_grad), score(_score), level(_pyramid_level), landmark(_landmark), seed_ref(_seed_ref), track_id(_track_id)
        {
            ;
        }

        FeatureWrapper() = delete;
        ~FeatureWrapper() = default;

        //! @todo (MWE) do copy and copy-asignment operators make sense?
        FeatureWrapper(const FeatureWrapper &other) = default;
        FeatureWrapper &operator=(const FeatureWrapper &other) = default;
    };

} // namespace mivins
