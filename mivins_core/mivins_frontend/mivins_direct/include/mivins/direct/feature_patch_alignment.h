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

#ifndef SVO_DIRECT_FEATURE_ALIGNMENT_H_
#define SVO_DIRECT_FEATURE_ALIGNMENT_H_

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <mivins/common/types.h>
#include <mivins/common/frame.h>
namespace mivins
{

    /// Subpixel refinement of a reference feature patch with the current image.
    /// Implements the inverse-compositional approach (see "Lucas-Kanade 20 Years on"
    /// paper by Baker.
    namespace feature_patch_alignment
    {

        bool Align1D(
            const cv::Mat &cur_img,
            const Eigen::Ref<GradientVector> &dir, // direction in which the patch is allowed to move
            uint8_t *ref_patch_with_border,
            uint8_t *ref_patch,
            const int n_iter,
            const bool affine_est_offset,
            const bool affine_est_gain,
            Keypoint *cur_px_estimate,
            AffLight cur_aff,
            double cur_exp_time,
            AffLight ref_aff,
            double ref_exp_time,
            double *h_inv = nullptr);

        bool Align1D_NEON (
            const cv::Mat& cur_img,
            const Eigen::Ref<GradientVector>& dir,
            uint8_t* ref_patch_with_border,
            uint8_t* ref_patch,
            const int n_iter,
            const bool affine_est_offset,
            const bool affine_est_gain,
            Keypoint* cur_px_estimate,
            AffLight cur_aff,
            double cur_exp_time,
            AffLight ref_aff,
            double ref_exp_time,
            double* h_inv);

        bool Align2D(
            const cv::Mat &cur_img,
            uint8_t *ref_patch_with_border,
            uint8_t *ref_patch,
            const int n_iter,
            const bool affine_est_offset,
            const bool affine_est_gain,
            Keypoint &cur_px_estimate,
            AffLight cur_aff,
            double cur_exp_time,
            AffLight ref_aff,
            double ref_exp_time,
            bool no_simd = false,
            std::vector<Eigen::Vector2f> *each_step = nullptr);

        bool Align2D_SSE2(
            const cv::Mat &cur_img,
            uint8_t *ref_patch_with_border,
            uint8_t *ref_patch,
            const int n_iter,
            Keypoint &cur_px_estimate);

        bool Align2D_NEON(
            const cv::Mat &cur_img,
            uint8_t *ref_patch_with_border,
            uint8_t *ref_patch,
            const int n_iter,
            Keypoint &cur_px_estimate);

        void AlignPyr2DVec(
            const std::vector<cv::Mat> &img_pyr_ref,
            const std::vector<cv::Mat> &img_pyr_cur,
            const int max_level,
            const int min_level,
            const std::vector<int> &patch_sizes,
            const int n_iter,
            const float min_update_squared,
            const std::vector<cv::Point2f> &px_ref,
            std::vector<cv::Point2f> &px_cur,
            std::vector<uint8_t> &status);

        bool AlignPyr2D(
            const std::vector<cv::Mat> &img_pyr_ref,
            const std::vector<cv::Mat> &img_pyr_cur,
            const int max_level,
            const int min_level,
            const std::vector<int> &patch_sizes,
            const int n_iter,
            const float min_update_squared,
            const Eigen::Vector2i &px_ref_level_0,
            Keypoint &px_cur_level_0);

    } // namespace feature_patch_alignment
} // namespace mivins

#endif // SVO_DIRECT_FEATURE_ALIGNMENT_H_
