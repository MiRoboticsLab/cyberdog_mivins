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
#include <mivins/common/camera_fwd.h>
#include <mivins/common/transformation.h>

namespace mivins
{

    // Forward declarations.
    class FeatureWrapper;

    /// Warp a patch from the reference view to the current view.
    namespace warp
    {

        using AffineTransformation2 = Eigen::Matrix2d;

        void GetWarpAffineMatrix(
            const CameraPtr &cam_ref,
            const CameraPtr &cam_cur,
            const Eigen::Ref<Keypoint> &px_ref,
            const Eigen::Ref<BearingVector> &f_ref,
            const double depth_ref,
            const Transformation &T_cur_ref,
            const int level_ref,
            AffineTransformation2 *A_cur_ref);

        void GetWarpMatrixAffineHomography(
            const CameraPtr &cam_ref,
            const CameraPtr &cam_cur,
            const Keypoint &px_ref,
            const BearingVector &f_ref,
            const BearingVector &normal_ref,
            const double depth_ref,
            const Transformation T_cur_ref,
            const int level_ref,
            AffineTransformation2 &A_cur_ref);

        int GetBestSearchLevel(
            const AffineTransformation2 &A_cur_ref,
            const int max_level);

        bool DoWarpAffine(
            const AffineTransformation2 &A_cur_ref,
            const cv::Mat &img_ref,
            const Eigen::Ref<Keypoint> &px_ref,
            const int level_ref,
            const int level_cur,
            const int halfpatch_size,
            uint8_t *patch);

        bool WarpPixelwise(
            const Frame &cur_frame,
            const Frame &ref_frame,
            const FeatureWrapper &ref_ftr,
            const int level_ref,
            const int level_cur,
            const int halfpatch_size,
            uint8_t *patch);

        void CreatePatchNoWarp(
            const cv::Mat &img,
            const Eigen::Vector2i &px,
            const int halfpatch_size,
            uint8_t *patch);

        void CreatePatchNoWarpInterpolated(
            const cv::Mat &img,
            const Eigen::Ref<Keypoint> &px,
            const int halfpatch_size,
            uint8_t *patch);

    } // namespace warp
} // namespace mivins
