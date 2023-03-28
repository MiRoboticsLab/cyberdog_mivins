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

#ifndef FAST_H
#define FAST_H

#include <vector>

namespace fast
{

    using ::std::vector;

    struct Fastxy
    {
        short x, y;
        Fastxy(short x_, short y_) : x(x_), y(y_) {}
    };

    typedef unsigned char fast_byte;

    /// NEON optimized version of the corner 9
    void FastCornerDetect9Neon(const fast_byte *img, int imgWidth, int imgHeight, int widthStep, short barrier, std::vector<Fastxy> &corners);

    /// SSE2 optimized version of the corner 9
    //void FastCornerDetect9Sse2(const fast_byte* img, int imgWidth, int imgHeight, int widthStep, short barrier, std::vector<Fastxy>& corners);

    /// plain C++ version of the corner 9
    void FastCornerDetect9(const fast_byte *img, int imgWidth, int imgHeight, int widthStep, short barrier, std::vector<Fastxy> &corners);

    /// NEON optimized version of the corner 10
    //void fast_corner_detect_10_neon(const fast_byte* img, int imgWidth, int imgHeight, int widthStep, short barrier, std::vector<Fastxy>& corners);

    /// SSE2 optimized version of the corner 10
    void FastCornerDetect10Sse2(const fast_byte *img, int imgWidth, int imgHeight, int widthStep, short barrier, std::vector<Fastxy> &corners);

    /// plain C++ version of the corner 10
    void FastCornerDetect10(const fast_byte *img, int imgWidth, int imgHeight, int widthStep, short barrier, std::vector<Fastxy> &corners);

    /// corner score 10
    void FastCornerScore10(const fast_byte *img, const int img_stride, const std::vector<Fastxy> &corners, const int threshold, std::vector<int> &scores);

    /// Nonmax Suppression on a 3x3 Window
    void FastNonmax3x3(const std::vector<Fastxy> &corners, const std::vector<int> &scores, std::vector<int> &nonmax_corners);

    /*
// It's either a dark-center-bright-background (DARK = true) corner,
// or a bright-center-on-dark-background (DARK = false) corner.
// Don't mix them!
template<bool DARK = true>
struct fast_corner
{
   fast_corner()
      : x(-1), y(-1), score(-1) {}
   fast_corner(const Fastxy& corner, short score_)
      : x(corner.x), y(corner.y), score(score_) {}

   short x, y;
   short score;
};

typedef fast_corner<true> DarkCorner;
typedef fast_corner<false> BrightCorner;
typedef std::vector<DarkCorner> DarkCorners;
typedef std::vector<BrightCorner> BrightCorners;


/// Nonmax Suppression on a 5x5 window
void FastNonmax5x5(const fast_byte* img, int imgWidth, int imgHeight, int widthStep, const std::vector<Fastxy>& corners,
                     short barrier, DarkCorners& darkCorners, BrightCorners& brightCorners);

/// special corner score. Sum of the absolute radiometric differences between the center pixel and the pixels on the circle.
short CornerScoreTest(const fast_byte* img, const int *pointer_dir, short barrier, bool* type);
*/

} // namespace Fast

#endif
