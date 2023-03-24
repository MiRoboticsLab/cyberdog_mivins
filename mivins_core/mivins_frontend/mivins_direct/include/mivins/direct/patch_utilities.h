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

#include <glog/logging.h>
#include <mivins/common/types.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace mivins
{
    namespace patch_utils
    {

        inline void CreatePatchFromPatchWithBorder(
            const uint8_t *const patch_with_border,
            const int patch_size,
            uint8_t *patch)
        {
            uint8_t *patch_ptr = patch;
            for (int y = 1; y < patch_size + 1; ++y, patch_ptr += patch_size)
            {
                const uint8_t *ref_patch_border_ptr = patch_with_border + y * (patch_size + 2) + 1;
                for (int x = 0; x < patch_size; ++x)
                    patch_ptr[x] = ref_patch_border_ptr[x];
            }
        }

        inline void PatchToMat(
            const uint8_t *const patch_data,
            const size_t patch_width,
            cv::Mat *img)
        {
            CHECK_NOTNULL(img);
            *img = cv::Mat(patch_width, patch_width, CV_8UC1);
            std::memcpy(img->data, patch_data, patch_width * patch_width);
        }

        inline void NormalizeAndUpsamplePatch(
            const cv::Mat &patch,
            double upsample_factor,
            cv::Mat *img_rgb)
        {
            CHECK_NOTNULL(img_rgb);

            cv::Mat patch_normalized;
            if (patch.type() == CV_32FC1)
            {
                double minval, maxval;
                cv::minMaxLoc(patch, &minval, &maxval);
                patch_normalized = (patch - minval) / (maxval - minval);
            }
            else if (patch.type() == CV_8UC1)
            {
                patch.convertTo(patch_normalized, CV_32FC1);
                double minval, maxval;
                cv::minMaxLoc(patch_normalized, &minval, &maxval);
                patch_normalized = (patch_normalized - minval) / (maxval - minval);
            }
            else
            {
                LOG(FATAL) << "Image Type not supported.";
            }
            cv::Mat img_gray;
            cv::resize(patch_normalized, img_gray, cv::Size(0, 0), upsample_factor, upsample_factor, cv::INTER_NEAREST);
            *img_rgb = cv::Mat(img_gray.size(), CV_8UC3);
            cv::cvtColor(img_gray, *img_rgb, cv::COLOR_GRAY2RGB);
        }

        inline void ConcatenatePatches(
            std::vector<cv::Mat> patches,
            cv::Mat *result_rgb)
        {
            const size_t n = patches.size();
            const int width = patches.at(0).cols;
            const int height = patches.at(0).rows;

            *result_rgb = cv::Mat(height, n * width, CV_32FC3);
            for (size_t i = 0; i < n; ++i)
            {
                CHECK_EQ(width, patches[i].cols);
                CHECK_EQ(height, patches[i].rows);
                cv::Mat roi(*result_rgb, cv::Rect(i * width, 0, width, height));
                cv::Mat patch_rgb(patches[i].size(), CV_32FC3);
                cv::cvtColor(patches[i], patch_rgb, cv::COLOR_GRAY2RGB);
                patch_rgb.copyTo(roi);
            }
        }

    } // namespace patch_utils
} // namespace mivins
