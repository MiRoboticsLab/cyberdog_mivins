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

#ifndef CV_UTILS_H_
#define CV_UTILS_H_

#include <Eigen/Core>

#include <opencv2/core/core.hpp>

namespace mivins
{

    /// Check if memory is 8bit aligned.
    inline bool IsAligned8Bit(const void *ptr)
    {
        return ((reinterpret_cast<size_t>(ptr)) & 0x7) == 0;
    }

    /// Check if memory is 16bit aligned.
    inline bool IsAligned16Bit(const void *ptr)
    {
        return ((reinterpret_cast<size_t>(ptr)) & 0xF) == 0;
    }

    //! Return value between 0 and 1
    //! WARNING This function does not check whether the x/y is within the border
    template<class T>
    inline float
    interpolateMat(const cv::Mat &mat, float u, float v)
    {
        assert(mat.type() == CV_32F);
        float x = floor(u);
        float y = floor(v);
        float subpix_x = u - x;
        float subpix_y = v - y;
        float wx0 = 1.0 - subpix_x;
        float wx1 = subpix_x;
        float wy0 = 1.0 - subpix_y;
        float wy1 = subpix_y;

        T val00 = mat.at<T>(y, x);
        T val10 = mat.at<T>(y, x + 1);
        T val01 = mat.at<T>(y + 1, x);
        T val11 = mat.at<T>(y + 1, x + 1);
        return (wx0 * wy0) * val00 + (wx1 * wy0) * val10 + (wx0 * wy1) * val01 + (wx1 * wy1) * val11;
    }

    void HalfSample(const cv::Mat &in, cv::Mat &out);

    void CalcSharrDeriv(const cv::Mat &src, cv::Mat &dst);

#ifdef __SSE2__

    /// Used to convert a Kinect depthmap
    /// Code by Christian Kerl DVO, GPL Licence
    void convertRawDepthImageSse_16u_to_32f(cv::Mat &depth_16u, cv::Mat &depth_32f, float scale);

#endif

} // namespace mivins

#endif // CV_UTILS_H_
