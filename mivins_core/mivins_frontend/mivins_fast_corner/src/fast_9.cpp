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

#include <cstdlib>
#include <fast/fast.h>
#include <vector>

namespace fast
{

    static void MakeOffsets(int pixel[], int row_stride)
    {
        pixel[0] = 0 + row_stride * 3;
        pixel[1] = 1 + row_stride * 3;
        pixel[2] = 2 + row_stride * 2;
        pixel[3] = 3 + row_stride * 1;
        pixel[4] = 3 + row_stride * 0;
        pixel[5] = 3 + row_stride * -1;
        pixel[6] = 2 + row_stride * -2;
        pixel[7] = 1 + row_stride * -3;
        pixel[8] = 0 + row_stride * -3;
        pixel[9] = -1 + row_stride * -3;
        pixel[10] = -2 + row_stride * -2;
        pixel[11] = -3 + row_stride * -1;
        pixel[12] = -3 + row_stride * 0;
        pixel[13] = -3 + row_stride * 1;
        pixel[14] = -2 + row_stride * 2;
        pixel[15] = -1 + row_stride * 3;
    }

    void FastCornerDetect9(const fast_byte *img, int imgWidth, int imgHeight, int widthStep,
                           short barrier, std::vector<Fastxy> &corners)
    {
        int pixel[16];
        int x, y;

        MakeOffsets(pixel, widthStep);

        for (y = 3; y < imgHeight - 3; ++y)
            for (x = 3; x < imgWidth - 3; ++x)
            {
                const fast_byte *p = img + y * widthStep + x;
                const short cb = *p + barrier;
                const short c_b = *p - barrier;
                if (p[pixel[0]] > cb)
                    if (p[pixel[1]] > cb)
                        if (p[pixel[2]] > cb)
                            if (p[pixel[3]] > cb)
                                if (p[pixel[4]] > cb)
                                    if (p[pixel[5]] > cb)
                                        if (p[pixel[6]] > cb)
                                            if (p[pixel[7]] > cb)
                                                if (p[pixel[8]] > cb)
                                                {
                                                }
                                                else if (p[pixel[15]] > cb)
                                                {
                                                }
                                                else
                                                    continue;
                                            else if (p[pixel[7]] < c_b)
                                                if (p[pixel[14]] > cb)
                                                    if (p[pixel[15]] > cb)
                                                    {
                                                    }
                                                    else
                                                        continue;
                                                else if (p[pixel[14]] < c_b)
                                                    if (p[pixel[8]] < c_b)
                                                        if (p[pixel[9]] < c_b)
                                                            if (p[pixel[10]] < c_b)
                                                                if (p[pixel[11]] < c_b)
                                                                    if (p[pixel[12]] < c_b)
                                                                        if (p[pixel[13]] < c_b)
                                                                            if (p[pixel[15]] < c_b)
                                                                            {
                                                                            }
                                                                            else
                                                                                continue;
                                                                        else
                                                                            continue;
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[14]] > cb)
                                                if (p[pixel[15]] > cb)
                                                {
                                                }
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[6]] < c_b)
                                            if (p[pixel[15]] > cb)
                                                if (p[pixel[13]] > cb)
                                                    if (p[pixel[14]] > cb)
                                                    {
                                                    }
                                                    else
                                                        continue;
                                                else if (p[pixel[13]] < c_b)
                                                    if (p[pixel[7]] < c_b)
                                                        if (p[pixel[8]] < c_b)
                                                            if (p[pixel[9]] < c_b)
                                                                if (p[pixel[10]] < c_b)
                                                                    if (p[pixel[11]] < c_b)
                                                                        if (p[pixel[12]] < c_b)
                                                                            if (p[pixel[14]] < c_b)
                                                                            {
                                                                            }
                                                                            else
                                                                                continue;
                                                                        else
                                                                            continue;
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[7]] < c_b)
                                                if (p[pixel[8]] < c_b)
                                                    if (p[pixel[9]] < c_b)
                                                        if (p[pixel[10]] < c_b)
                                                            if (p[pixel[11]] < c_b)
                                                                if (p[pixel[12]] < c_b)
                                                                    if (p[pixel[13]] < c_b)
                                                                        if (p[pixel[14]] < c_b)
                                                                        {
                                                                        }
                                                                        else
                                                                            continue;
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[13]] > cb)
                                            if (p[pixel[14]] > cb)
                                                if (p[pixel[15]] > cb)
                                                {
                                                }
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[13]] < c_b)
                                            if (p[pixel[7]] < c_b)
                                                if (p[pixel[8]] < c_b)
                                                    if (p[pixel[9]] < c_b)
                                                        if (p[pixel[10]] < c_b)
                                                            if (p[pixel[11]] < c_b)
                                                                if (p[pixel[12]] < c_b)
                                                                    if (p[pixel[14]] < c_b)
                                                                        if (p[pixel[15]] < c_b)
                                                                        {
                                                                        }
                                                                        else
                                                                            continue;
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if (p[pixel[5]] < c_b)
                                        if (p[pixel[14]] > cb)
                                            if (p[pixel[12]] > cb)
                                                if (p[pixel[13]] > cb)
                                                    if (p[pixel[15]] > cb)
                                                    {
                                                    }
                                                    else if (p[pixel[6]] > cb)
                                                        if (p[pixel[7]] > cb)
                                                            if (p[pixel[8]] > cb)
                                                                if (p[pixel[9]] > cb)
                                                                    if (p[pixel[10]] > cb)
                                                                        if (p[pixel[11]] > cb)
                                                                        {
                                                                        }
                                                                        else
                                                                            continue;
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[12]] < c_b)
                                                if (p[pixel[6]] < c_b)
                                                    if (p[pixel[7]] < c_b)
                                                        if (p[pixel[8]] < c_b)
                                                            if (p[pixel[9]] < c_b)
                                                                if (p[pixel[10]] < c_b)
                                                                    if (p[pixel[11]] < c_b)
                                                                        if (p[pixel[13]] < c_b)
                                                                        {
                                                                        }
                                                                        else
                                                                            continue;
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[14]] < c_b)
                                            if (p[pixel[7]] < c_b)
                                                if (p[pixel[8]] < c_b)
                                                    if (p[pixel[9]] < c_b)
                                                        if (p[pixel[10]] < c_b)
                                                            if (p[pixel[11]] < c_b)
                                                                if (p[pixel[12]] < c_b)
                                                                    if (p[pixel[13]] < c_b)
                                                                        if (p[pixel[6]] < c_b)
                                                                        {
                                                                        }
                                                                        else if (p[pixel[15]] < c_b)
                                                                        {
                                                                        }
                                                                        else
                                                                            continue;
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[6]] < c_b)
                                            if (p[pixel[7]] < c_b)
                                                if (p[pixel[8]] < c_b)
                                                    if (p[pixel[9]] < c_b)
                                                        if (p[pixel[10]] < c_b)
                                                            if (p[pixel[11]] < c_b)
                                                                if (p[pixel[12]] < c_b)
                                                                    if (p[pixel[13]] < c_b)
                                                                    {
                                                                    }
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if (p[pixel[12]] > cb)
                                        if (p[pixel[13]] > cb)
                                            if (p[pixel[14]] > cb)
                                                if (p[pixel[15]] > cb)
                                                {
                                                }
                                                else if (p[pixel[6]] > cb)
                                                    if (p[pixel[7]] > cb)
                                                        if (p[pixel[8]] > cb)
                                                            if (p[pixel[9]] > cb)
                                                                if (p[pixel[10]] > cb)
                                                                    if (p[pixel[11]] > cb)
                                                                    {
                                                                    }
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if (p[pixel[12]] < c_b)
                                        if (p[pixel[7]] < c_b)
                                            if (p[pixel[8]] < c_b)
                                                if (p[pixel[9]] < c_b)
                                                    if (p[pixel[10]] < c_b)
                                                        if (p[pixel[11]] < c_b)
                                                            if (p[pixel[13]] < c_b)
                                                                if (p[pixel[14]] < c_b)
                                                                    if (p[pixel[6]] < c_b)
                                                                    {
                                                                    }
                                                                    else if (p[pixel[15]] < c_b)
                                                                    {
                                                                    }
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if (p[pixel[4]] < c_b)
                                    if (p[pixel[13]] > cb)
                                        if (p[pixel[11]] > cb)
                                            if (p[pixel[12]] > cb)
                                                if (p[pixel[14]] > cb)
                                                    if (p[pixel[15]] > cb)
                                                    {
                                                    }
                                                    else if (p[pixel[6]] > cb)
                                                        if (p[pixel[7]] > cb)
                                                            if (p[pixel[8]] > cb)
                                                                if (p[pixel[9]] > cb)
                                                                    if (p[pixel[10]] > cb)
                                                                    {
                                                                    }
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else if (p[pixel[5]] > cb)
                                                    if (p[pixel[6]] > cb)
                                                        if (p[pixel[7]] > cb)
                                                            if (p[pixel[8]] > cb)
                                                                if (p[pixel[9]] > cb)
                                                                    if (p[pixel[10]] > cb)
                                                                    {
                                                                    }
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[11]] < c_b)
                                            if (p[pixel[5]] < c_b)
                                                if (p[pixel[6]] < c_b)
                                                    if (p[pixel[7]] < c_b)
                                                        if (p[pixel[8]] < c_b)
                                                            if (p[pixel[9]] < c_b)
                                                                if (p[pixel[10]] < c_b)
                                                                    if (p[pixel[12]] < c_b)
                                                                    {
                                                                    }
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if (p[pixel[13]] < c_b)
                                        if (p[pixel[7]] < c_b)
                                            if (p[pixel[8]] < c_b)
                                                if (p[pixel[9]] < c_b)
                                                    if (p[pixel[10]] < c_b)
                                                        if (p[pixel[11]] < c_b)
                                                            if (p[pixel[12]] < c_b)
                                                                if (p[pixel[6]] < c_b)
                                                                    if (p[pixel[5]] < c_b)
                                                                    {
                                                                    }
                                                                    else if (p[pixel[14]] < c_b)
                                                                    {
                                                                    }
                                                                    else
                                                                        continue;
                                                                else if (p[pixel[14]] < c_b)
                                                                    if (p[pixel[15]] < c_b)
                                                                    {
                                                                    }
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if (p[pixel[5]] < c_b)
                                        if (p[pixel[6]] < c_b)
                                            if (p[pixel[7]] < c_b)
                                                if (p[pixel[8]] < c_b)
                                                    if (p[pixel[9]] < c_b)
                                                        if (p[pixel[10]] < c_b)
                                                            if (p[pixel[11]] < c_b)
                                                                if (p[pixel[12]] < c_b)
                                                                {
                                                                }
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if (p[pixel[11]] > cb)
                                    if (p[pixel[12]] > cb)
                                        if (p[pixel[13]] > cb)
                                            if (p[pixel[14]] > cb)
                                                if (p[pixel[15]] > cb)
                                                {
                                                }
                                                else if (p[pixel[6]] > cb)
                                                    if (p[pixel[7]] > cb)
                                                        if (p[pixel[8]] > cb)
                                                            if (p[pixel[9]] > cb)
                                                                if (p[pixel[10]] > cb)
                                                                {
                                                                }
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[5]] > cb)
                                                if (p[pixel[6]] > cb)
                                                    if (p[pixel[7]] > cb)
                                                        if (p[pixel[8]] > cb)
                                                            if (p[pixel[9]] > cb)
                                                                if (p[pixel[10]] > cb)
                                                                {
                                                                }
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if (p[pixel[11]] < c_b)
                                    if (p[pixel[7]] < c_b)
                                        if (p[pixel[8]] < c_b)
                                            if (p[pixel[9]] < c_b)
                                                if (p[pixel[10]] < c_b)
                                                    if (p[pixel[12]] < c_b)
                                                        if (p[pixel[13]] < c_b)
                                                            if (p[pixel[6]] < c_b)
                                                                if (p[pixel[5]] < c_b)
                                                                {
                                                                }
                                                                else if (p[pixel[14]] < c_b)
                                                                {
                                                                }
                                                                else
                                                                    continue;
                                                            else if (p[pixel[14]] < c_b)
                                                                if (p[pixel[15]] < c_b)
                                                                {
                                                                }
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else if (p[pixel[3]] < c_b)
                                if (p[pixel[10]] > cb)
                                    if (p[pixel[11]] > cb)
                                        if (p[pixel[12]] > cb)
                                            if (p[pixel[13]] > cb)
                                                if (p[pixel[14]] > cb)
                                                    if (p[pixel[15]] > cb)
                                                    {
                                                    }
                                                    else if (p[pixel[6]] > cb)
                                                        if (p[pixel[7]] > cb)
                                                            if (p[pixel[8]] > cb)
                                                                if (p[pixel[9]] > cb)
                                                                {
                                                                }
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else if (p[pixel[5]] > cb)
                                                    if (p[pixel[6]] > cb)
                                                        if (p[pixel[7]] > cb)
                                                            if (p[pixel[8]] > cb)
                                                                if (p[pixel[9]] > cb)
                                                                {
                                                                }
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[4]] > cb)
                                                if (p[pixel[5]] > cb)
                                                    if (p[pixel[6]] > cb)
                                                        if (p[pixel[7]] > cb)
                                                            if (p[pixel[8]] > cb)
                                                                if (p[pixel[9]] > cb)
                                                                {
                                                                }
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if (p[pixel[10]] < c_b)
                                    if (p[pixel[7]] < c_b)
                                        if (p[pixel[8]] < c_b)
                                            if (p[pixel[9]] < c_b)
                                                if (p[pixel[11]] < c_b)
                                                    if (p[pixel[6]] < c_b)
                                                        if (p[pixel[5]] < c_b)
                                                            if (p[pixel[4]] < c_b)
                                                            {
                                                            }
                                                            else if (p[pixel[12]] < c_b)
                                                                if (p[pixel[13]] < c_b)
                                                                {
                                                                }
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else if (p[pixel[12]] < c_b)
                                                            if (p[pixel[13]] < c_b)
                                                                if (p[pixel[14]] < c_b)
                                                                {
                                                                }
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else if (p[pixel[12]] < c_b)
                                                        if (p[pixel[13]] < c_b)
                                                            if (p[pixel[14]] < c_b)
                                                                if (p[pixel[15]] < c_b)
                                                                {
                                                                }
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else if (p[pixel[10]] > cb)
                                if (p[pixel[11]] > cb)
                                    if (p[pixel[12]] > cb)
                                        if (p[pixel[13]] > cb)
                                            if (p[pixel[14]] > cb)
                                                if (p[pixel[15]] > cb)
                                                {
                                                }
                                                else if (p[pixel[6]] > cb)
                                                    if (p[pixel[7]] > cb)
                                                        if (p[pixel[8]] > cb)
                                                            if (p[pixel[9]] > cb)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[5]] > cb)
                                                if (p[pixel[6]] > cb)
                                                    if (p[pixel[7]] > cb)
                                                        if (p[pixel[8]] > cb)
                                                            if (p[pixel[9]] > cb)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[4]] > cb)
                                            if (p[pixel[5]] > cb)
                                                if (p[pixel[6]] > cb)
                                                    if (p[pixel[7]] > cb)
                                                        if (p[pixel[8]] > cb)
                                                            if (p[pixel[9]] > cb)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else if (p[pixel[10]] < c_b)
                                if (p[pixel[7]] < c_b)
                                    if (p[pixel[8]] < c_b)
                                        if (p[pixel[9]] < c_b)
                                            if (p[pixel[11]] < c_b)
                                                if (p[pixel[12]] < c_b)
                                                    if (p[pixel[6]] < c_b)
                                                        if (p[pixel[5]] < c_b)
                                                            if (p[pixel[4]] < c_b)
                                                            {
                                                            }
                                                            else if (p[pixel[13]] < c_b)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else if (p[pixel[13]] < c_b)
                                                            if (p[pixel[14]] < c_b)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else if (p[pixel[13]] < c_b)
                                                        if (p[pixel[14]] < c_b)
                                                            if (p[pixel[15]] < c_b)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else if (p[pixel[2]] < c_b)
                            if (p[pixel[9]] > cb)
                                if (p[pixel[10]] > cb)
                                    if (p[pixel[11]] > cb)
                                        if (p[pixel[12]] > cb)
                                            if (p[pixel[13]] > cb)
                                                if (p[pixel[14]] > cb)
                                                    if (p[pixel[15]] > cb)
                                                    {
                                                    }
                                                    else if (p[pixel[6]] > cb)
                                                        if (p[pixel[7]] > cb)
                                                            if (p[pixel[8]] > cb)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else if (p[pixel[5]] > cb)
                                                    if (p[pixel[6]] > cb)
                                                        if (p[pixel[7]] > cb)
                                                            if (p[pixel[8]] > cb)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[4]] > cb)
                                                if (p[pixel[5]] > cb)
                                                    if (p[pixel[6]] > cb)
                                                        if (p[pixel[7]] > cb)
                                                            if (p[pixel[8]] > cb)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[3]] > cb)
                                            if (p[pixel[4]] > cb)
                                                if (p[pixel[5]] > cb)
                                                    if (p[pixel[6]] > cb)
                                                        if (p[pixel[7]] > cb)
                                                            if (p[pixel[8]] > cb)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else if (p[pixel[9]] < c_b)
                                if (p[pixel[7]] < c_b)
                                    if (p[pixel[8]] < c_b)
                                        if (p[pixel[10]] < c_b)
                                            if (p[pixel[6]] < c_b)
                                                if (p[pixel[5]] < c_b)
                                                    if (p[pixel[4]] < c_b)
                                                        if (p[pixel[3]] < c_b)
                                                        {
                                                        }
                                                        else if (p[pixel[11]] < c_b)
                                                            if (p[pixel[12]] < c_b)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else if (p[pixel[11]] < c_b)
                                                        if (p[pixel[12]] < c_b)
                                                            if (p[pixel[13]] < c_b)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else if (p[pixel[11]] < c_b)
                                                    if (p[pixel[12]] < c_b)
                                                        if (p[pixel[13]] < c_b)
                                                            if (p[pixel[14]] < c_b)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[11]] < c_b)
                                                if (p[pixel[12]] < c_b)
                                                    if (p[pixel[13]] < c_b)
                                                        if (p[pixel[14]] < c_b)
                                                            if (p[pixel[15]] < c_b)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else if (p[pixel[9]] > cb)
                            if (p[pixel[10]] > cb)
                                if (p[pixel[11]] > cb)
                                    if (p[pixel[12]] > cb)
                                        if (p[pixel[13]] > cb)
                                            if (p[pixel[14]] > cb)
                                                if (p[pixel[15]] > cb)
                                                {
                                                }
                                                else if (p[pixel[6]] > cb)
                                                    if (p[pixel[7]] > cb)
                                                        if (p[pixel[8]] > cb)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[5]] > cb)
                                                if (p[pixel[6]] > cb)
                                                    if (p[pixel[7]] > cb)
                                                        if (p[pixel[8]] > cb)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[4]] > cb)
                                            if (p[pixel[5]] > cb)
                                                if (p[pixel[6]] > cb)
                                                    if (p[pixel[7]] > cb)
                                                        if (p[pixel[8]] > cb)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if (p[pixel[3]] > cb)
                                        if (p[pixel[4]] > cb)
                                            if (p[pixel[5]] > cb)
                                                if (p[pixel[6]] > cb)
                                                    if (p[pixel[7]] > cb)
                                                        if (p[pixel[8]] > cb)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else if (p[pixel[9]] < c_b)
                            if (p[pixel[7]] < c_b)
                                if (p[pixel[8]] < c_b)
                                    if (p[pixel[10]] < c_b)
                                        if (p[pixel[11]] < c_b)
                                            if (p[pixel[6]] < c_b)
                                                if (p[pixel[5]] < c_b)
                                                    if (p[pixel[4]] < c_b)
                                                        if (p[pixel[3]] < c_b)
                                                        {
                                                        }
                                                        else if (p[pixel[12]] < c_b)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else if (p[pixel[12]] < c_b)
                                                        if (p[pixel[13]] < c_b)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else if (p[pixel[12]] < c_b)
                                                    if (p[pixel[13]] < c_b)
                                                        if (p[pixel[14]] < c_b)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[12]] < c_b)
                                                if (p[pixel[13]] < c_b)
                                                    if (p[pixel[14]] < c_b)
                                                        if (p[pixel[15]] < c_b)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else if (p[pixel[1]] < c_b)
                        if (p[pixel[8]] > cb)
                            if (p[pixel[9]] > cb)
                                if (p[pixel[10]] > cb)
                                    if (p[pixel[11]] > cb)
                                        if (p[pixel[12]] > cb)
                                            if (p[pixel[13]] > cb)
                                                if (p[pixel[14]] > cb)
                                                    if (p[pixel[15]] > cb)
                                                    {
                                                    }
                                                    else if (p[pixel[6]] > cb)
                                                        if (p[pixel[7]] > cb)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else if (p[pixel[5]] > cb)
                                                    if (p[pixel[6]] > cb)
                                                        if (p[pixel[7]] > cb)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[4]] > cb)
                                                if (p[pixel[5]] > cb)
                                                    if (p[pixel[6]] > cb)
                                                        if (p[pixel[7]] > cb)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[3]] > cb)
                                            if (p[pixel[4]] > cb)
                                                if (p[pixel[5]] > cb)
                                                    if (p[pixel[6]] > cb)
                                                        if (p[pixel[7]] > cb)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if (p[pixel[2]] > cb)
                                        if (p[pixel[3]] > cb)
                                            if (p[pixel[4]] > cb)
                                                if (p[pixel[5]] > cb)
                                                    if (p[pixel[6]] > cb)
                                                        if (p[pixel[7]] > cb)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else if (p[pixel[8]] < c_b)
                            if (p[pixel[7]] < c_b)
                                if (p[pixel[9]] < c_b)
                                    if (p[pixel[6]] < c_b)
                                        if (p[pixel[5]] < c_b)
                                            if (p[pixel[4]] < c_b)
                                                if (p[pixel[3]] < c_b)
                                                    if (p[pixel[2]] < c_b)
                                                    {
                                                    }
                                                    else if (p[pixel[10]] < c_b)
                                                        if (p[pixel[11]] < c_b)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else if (p[pixel[10]] < c_b)
                                                    if (p[pixel[11]] < c_b)
                                                        if (p[pixel[12]] < c_b)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[10]] < c_b)
                                                if (p[pixel[11]] < c_b)
                                                    if (p[pixel[12]] < c_b)
                                                        if (p[pixel[13]] < c_b)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[10]] < c_b)
                                            if (p[pixel[11]] < c_b)
                                                if (p[pixel[12]] < c_b)
                                                    if (p[pixel[13]] < c_b)
                                                        if (p[pixel[14]] < c_b)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if (p[pixel[10]] < c_b)
                                        if (p[pixel[11]] < c_b)
                                            if (p[pixel[12]] < c_b)
                                                if (p[pixel[13]] < c_b)
                                                    if (p[pixel[14]] < c_b)
                                                        if (p[pixel[15]] < c_b)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else if (p[pixel[8]] > cb)
                        if (p[pixel[9]] > cb)
                            if (p[pixel[10]] > cb)
                                if (p[pixel[11]] > cb)
                                    if (p[pixel[12]] > cb)
                                        if (p[pixel[13]] > cb)
                                            if (p[pixel[14]] > cb)
                                                if (p[pixel[15]] > cb)
                                                {
                                                }
                                                else if (p[pixel[6]] > cb)
                                                    if (p[pixel[7]] > cb)
                                                    {
                                                    }
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[5]] > cb)
                                                if (p[pixel[6]] > cb)
                                                    if (p[pixel[7]] > cb)
                                                    {
                                                    }
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[4]] > cb)
                                            if (p[pixel[5]] > cb)
                                                if (p[pixel[6]] > cb)
                                                    if (p[pixel[7]] > cb)
                                                    {
                                                    }
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if (p[pixel[3]] > cb)
                                        if (p[pixel[4]] > cb)
                                            if (p[pixel[5]] > cb)
                                                if (p[pixel[6]] > cb)
                                                    if (p[pixel[7]] > cb)
                                                    {
                                                    }
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if (p[pixel[2]] > cb)
                                    if (p[pixel[3]] > cb)
                                        if (p[pixel[4]] > cb)
                                            if (p[pixel[5]] > cb)
                                                if (p[pixel[6]] > cb)
                                                    if (p[pixel[7]] > cb)
                                                    {
                                                    }
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else if (p[pixel[8]] < c_b)
                        if (p[pixel[7]] < c_b)
                            if (p[pixel[9]] < c_b)
                                if (p[pixel[10]] < c_b)
                                    if (p[pixel[6]] < c_b)
                                        if (p[pixel[5]] < c_b)
                                            if (p[pixel[4]] < c_b)
                                                if (p[pixel[3]] < c_b)
                                                    if (p[pixel[2]] < c_b)
                                                    {
                                                    }
                                                    else if (p[pixel[11]] < c_b)
                                                    {
                                                    }
                                                    else
                                                        continue;
                                                else if (p[pixel[11]] < c_b)
                                                    if (p[pixel[12]] < c_b)
                                                    {
                                                    }
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[11]] < c_b)
                                                if (p[pixel[12]] < c_b)
                                                    if (p[pixel[13]] < c_b)
                                                    {
                                                    }
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[11]] < c_b)
                                            if (p[pixel[12]] < c_b)
                                                if (p[pixel[13]] < c_b)
                                                    if (p[pixel[14]] < c_b)
                                                    {
                                                    }
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if (p[pixel[11]] < c_b)
                                        if (p[pixel[12]] < c_b)
                                            if (p[pixel[13]] < c_b)
                                                if (p[pixel[14]] < c_b)
                                                    if (p[pixel[15]] < c_b)
                                                    {
                                                    }
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else
                        continue;
                else if (p[pixel[0]] < c_b)
                    if (p[pixel[1]] > cb)
                        if (p[pixel[8]] > cb)
                            if (p[pixel[7]] > cb)
                                if (p[pixel[9]] > cb)
                                    if (p[pixel[6]] > cb)
                                        if (p[pixel[5]] > cb)
                                            if (p[pixel[4]] > cb)
                                                if (p[pixel[3]] > cb)
                                                    if (p[pixel[2]] > cb)
                                                    {
                                                    }
                                                    else if (p[pixel[10]] > cb)
                                                        if (p[pixel[11]] > cb)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else if (p[pixel[10]] > cb)
                                                    if (p[pixel[11]] > cb)
                                                        if (p[pixel[12]] > cb)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[10]] > cb)
                                                if (p[pixel[11]] > cb)
                                                    if (p[pixel[12]] > cb)
                                                        if (p[pixel[13]] > cb)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[10]] > cb)
                                            if (p[pixel[11]] > cb)
                                                if (p[pixel[12]] > cb)
                                                    if (p[pixel[13]] > cb)
                                                        if (p[pixel[14]] > cb)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if (p[pixel[10]] > cb)
                                        if (p[pixel[11]] > cb)
                                            if (p[pixel[12]] > cb)
                                                if (p[pixel[13]] > cb)
                                                    if (p[pixel[14]] > cb)
                                                        if (p[pixel[15]] > cb)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else if (p[pixel[8]] < c_b)
                            if (p[pixel[9]] < c_b)
                                if (p[pixel[10]] < c_b)
                                    if (p[pixel[11]] < c_b)
                                        if (p[pixel[12]] < c_b)
                                            if (p[pixel[13]] < c_b)
                                                if (p[pixel[14]] < c_b)
                                                    if (p[pixel[15]] < c_b)
                                                    {
                                                    }
                                                    else if (p[pixel[6]] < c_b)
                                                        if (p[pixel[7]] < c_b)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else if (p[pixel[5]] < c_b)
                                                    if (p[pixel[6]] < c_b)
                                                        if (p[pixel[7]] < c_b)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[4]] < c_b)
                                                if (p[pixel[5]] < c_b)
                                                    if (p[pixel[6]] < c_b)
                                                        if (p[pixel[7]] < c_b)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[3]] < c_b)
                                            if (p[pixel[4]] < c_b)
                                                if (p[pixel[5]] < c_b)
                                                    if (p[pixel[6]] < c_b)
                                                        if (p[pixel[7]] < c_b)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if (p[pixel[2]] < c_b)
                                        if (p[pixel[3]] < c_b)
                                            if (p[pixel[4]] < c_b)
                                                if (p[pixel[5]] < c_b)
                                                    if (p[pixel[6]] < c_b)
                                                        if (p[pixel[7]] < c_b)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else if (p[pixel[1]] < c_b)
                        if (p[pixel[2]] > cb)
                            if (p[pixel[9]] > cb)
                                if (p[pixel[7]] > cb)
                                    if (p[pixel[8]] > cb)
                                        if (p[pixel[10]] > cb)
                                            if (p[pixel[6]] > cb)
                                                if (p[pixel[5]] > cb)
                                                    if (p[pixel[4]] > cb)
                                                        if (p[pixel[3]] > cb)
                                                        {
                                                        }
                                                        else if (p[pixel[11]] > cb)
                                                            if (p[pixel[12]] > cb)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else if (p[pixel[11]] > cb)
                                                        if (p[pixel[12]] > cb)
                                                            if (p[pixel[13]] > cb)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else if (p[pixel[11]] > cb)
                                                    if (p[pixel[12]] > cb)
                                                        if (p[pixel[13]] > cb)
                                                            if (p[pixel[14]] > cb)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[11]] > cb)
                                                if (p[pixel[12]] > cb)
                                                    if (p[pixel[13]] > cb)
                                                        if (p[pixel[14]] > cb)
                                                            if (p[pixel[15]] > cb)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else if (p[pixel[9]] < c_b)
                                if (p[pixel[10]] < c_b)
                                    if (p[pixel[11]] < c_b)
                                        if (p[pixel[12]] < c_b)
                                            if (p[pixel[13]] < c_b)
                                                if (p[pixel[14]] < c_b)
                                                    if (p[pixel[15]] < c_b)
                                                    {
                                                    }
                                                    else if (p[pixel[6]] < c_b)
                                                        if (p[pixel[7]] < c_b)
                                                            if (p[pixel[8]] < c_b)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else if (p[pixel[5]] < c_b)
                                                    if (p[pixel[6]] < c_b)
                                                        if (p[pixel[7]] < c_b)
                                                            if (p[pixel[8]] < c_b)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[4]] < c_b)
                                                if (p[pixel[5]] < c_b)
                                                    if (p[pixel[6]] < c_b)
                                                        if (p[pixel[7]] < c_b)
                                                            if (p[pixel[8]] < c_b)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[3]] < c_b)
                                            if (p[pixel[4]] < c_b)
                                                if (p[pixel[5]] < c_b)
                                                    if (p[pixel[6]] < c_b)
                                                        if (p[pixel[7]] < c_b)
                                                            if (p[pixel[8]] < c_b)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else if (p[pixel[2]] < c_b)
                            if (p[pixel[3]] > cb)
                                if (p[pixel[10]] > cb)
                                    if (p[pixel[7]] > cb)
                                        if (p[pixel[8]] > cb)
                                            if (p[pixel[9]] > cb)
                                                if (p[pixel[11]] > cb)
                                                    if (p[pixel[6]] > cb)
                                                        if (p[pixel[5]] > cb)
                                                            if (p[pixel[4]] > cb)
                                                            {
                                                            }
                                                            else if (p[pixel[12]] > cb)
                                                                if (p[pixel[13]] > cb)
                                                                {
                                                                }
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else if (p[pixel[12]] > cb)
                                                            if (p[pixel[13]] > cb)
                                                                if (p[pixel[14]] > cb)
                                                                {
                                                                }
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else if (p[pixel[12]] > cb)
                                                        if (p[pixel[13]] > cb)
                                                            if (p[pixel[14]] > cb)
                                                                if (p[pixel[15]] > cb)
                                                                {
                                                                }
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if (p[pixel[10]] < c_b)
                                    if (p[pixel[11]] < c_b)
                                        if (p[pixel[12]] < c_b)
                                            if (p[pixel[13]] < c_b)
                                                if (p[pixel[14]] < c_b)
                                                    if (p[pixel[15]] < c_b)
                                                    {
                                                    }
                                                    else if (p[pixel[6]] < c_b)
                                                        if (p[pixel[7]] < c_b)
                                                            if (p[pixel[8]] < c_b)
                                                                if (p[pixel[9]] < c_b)
                                                                {
                                                                }
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else if (p[pixel[5]] < c_b)
                                                    if (p[pixel[6]] < c_b)
                                                        if (p[pixel[7]] < c_b)
                                                            if (p[pixel[8]] < c_b)
                                                                if (p[pixel[9]] < c_b)
                                                                {
                                                                }
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[4]] < c_b)
                                                if (p[pixel[5]] < c_b)
                                                    if (p[pixel[6]] < c_b)
                                                        if (p[pixel[7]] < c_b)
                                                            if (p[pixel[8]] < c_b)
                                                                if (p[pixel[9]] < c_b)
                                                                {
                                                                }
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else if (p[pixel[3]] < c_b)
                                if (p[pixel[4]] > cb)
                                    if (p[pixel[13]] > cb)
                                        if (p[pixel[7]] > cb)
                                            if (p[pixel[8]] > cb)
                                                if (p[pixel[9]] > cb)
                                                    if (p[pixel[10]] > cb)
                                                        if (p[pixel[11]] > cb)
                                                            if (p[pixel[12]] > cb)
                                                                if (p[pixel[6]] > cb)
                                                                    if (p[pixel[5]] > cb)
                                                                    {
                                                                    }
                                                                    else if (p[pixel[14]] > cb)
                                                                    {
                                                                    }
                                                                    else
                                                                        continue;
                                                                else if (p[pixel[14]] > cb)
                                                                    if (p[pixel[15]] > cb)
                                                                    {
                                                                    }
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if (p[pixel[13]] < c_b)
                                        if (p[pixel[11]] > cb)
                                            if (p[pixel[5]] > cb)
                                                if (p[pixel[6]] > cb)
                                                    if (p[pixel[7]] > cb)
                                                        if (p[pixel[8]] > cb)
                                                            if (p[pixel[9]] > cb)
                                                                if (p[pixel[10]] > cb)
                                                                    if (p[pixel[12]] > cb)
                                                                    {
                                                                    }
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[11]] < c_b)
                                            if (p[pixel[12]] < c_b)
                                                if (p[pixel[14]] < c_b)
                                                    if (p[pixel[15]] < c_b)
                                                    {
                                                    }
                                                    else if (p[pixel[6]] < c_b)
                                                        if (p[pixel[7]] < c_b)
                                                            if (p[pixel[8]] < c_b)
                                                                if (p[pixel[9]] < c_b)
                                                                    if (p[pixel[10]] < c_b)
                                                                    {
                                                                    }
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else if (p[pixel[5]] < c_b)
                                                    if (p[pixel[6]] < c_b)
                                                        if (p[pixel[7]] < c_b)
                                                            if (p[pixel[8]] < c_b)
                                                                if (p[pixel[9]] < c_b)
                                                                    if (p[pixel[10]] < c_b)
                                                                    {
                                                                    }
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if (p[pixel[5]] > cb)
                                        if (p[pixel[6]] > cb)
                                            if (p[pixel[7]] > cb)
                                                if (p[pixel[8]] > cb)
                                                    if (p[pixel[9]] > cb)
                                                        if (p[pixel[10]] > cb)
                                                            if (p[pixel[11]] > cb)
                                                                if (p[pixel[12]] > cb)
                                                                {
                                                                }
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if (p[pixel[4]] < c_b)
                                    if (p[pixel[5]] > cb)
                                        if (p[pixel[14]] > cb)
                                            if (p[pixel[7]] > cb)
                                                if (p[pixel[8]] > cb)
                                                    if (p[pixel[9]] > cb)
                                                        if (p[pixel[10]] > cb)
                                                            if (p[pixel[11]] > cb)
                                                                if (p[pixel[12]] > cb)
                                                                    if (p[pixel[13]] > cb)
                                                                        if (p[pixel[6]] > cb)
                                                                        {
                                                                        }
                                                                        else if (p[pixel[15]] > cb)
                                                                        {
                                                                        }
                                                                        else
                                                                            continue;
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[14]] < c_b)
                                            if (p[pixel[12]] > cb)
                                                if (p[pixel[6]] > cb)
                                                    if (p[pixel[7]] > cb)
                                                        if (p[pixel[8]] > cb)
                                                            if (p[pixel[9]] > cb)
                                                                if (p[pixel[10]] > cb)
                                                                    if (p[pixel[11]] > cb)
                                                                        if (p[pixel[13]] > cb)
                                                                        {
                                                                        }
                                                                        else
                                                                            continue;
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[12]] < c_b)
                                                if (p[pixel[13]] < c_b)
                                                    if (p[pixel[15]] < c_b)
                                                    {
                                                    }
                                                    else if (p[pixel[6]] < c_b)
                                                        if (p[pixel[7]] < c_b)
                                                            if (p[pixel[8]] < c_b)
                                                                if (p[pixel[9]] < c_b)
                                                                    if (p[pixel[10]] < c_b)
                                                                        if (p[pixel[11]] < c_b)
                                                                        {
                                                                        }
                                                                        else
                                                                            continue;
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[6]] > cb)
                                            if (p[pixel[7]] > cb)
                                                if (p[pixel[8]] > cb)
                                                    if (p[pixel[9]] > cb)
                                                        if (p[pixel[10]] > cb)
                                                            if (p[pixel[11]] > cb)
                                                                if (p[pixel[12]] > cb)
                                                                    if (p[pixel[13]] > cb)
                                                                    {
                                                                    }
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if (p[pixel[5]] < c_b)
                                        if (p[pixel[6]] > cb)
                                            if (p[pixel[15]] < c_b)
                                                if (p[pixel[13]] > cb)
                                                    if (p[pixel[7]] > cb)
                                                        if (p[pixel[8]] > cb)
                                                            if (p[pixel[9]] > cb)
                                                                if (p[pixel[10]] > cb)
                                                                    if (p[pixel[11]] > cb)
                                                                        if (p[pixel[12]] > cb)
                                                                            if (p[pixel[14]] > cb)
                                                                            {
                                                                            }
                                                                            else
                                                                                continue;
                                                                        else
                                                                            continue;
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else if (p[pixel[13]] < c_b)
                                                    if (p[pixel[14]] < c_b)
                                                    {
                                                    }
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[7]] > cb)
                                                if (p[pixel[8]] > cb)
                                                    if (p[pixel[9]] > cb)
                                                        if (p[pixel[10]] > cb)
                                                            if (p[pixel[11]] > cb)
                                                                if (p[pixel[12]] > cb)
                                                                    if (p[pixel[13]] > cb)
                                                                        if (p[pixel[14]] > cb)
                                                                        {
                                                                        }
                                                                        else
                                                                            continue;
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[6]] < c_b)
                                            if (p[pixel[7]] > cb)
                                                if (p[pixel[14]] > cb)
                                                    if (p[pixel[8]] > cb)
                                                        if (p[pixel[9]] > cb)
                                                            if (p[pixel[10]] > cb)
                                                                if (p[pixel[11]] > cb)
                                                                    if (p[pixel[12]] > cb)
                                                                        if (p[pixel[13]] > cb)
                                                                            if (p[pixel[15]] > cb)
                                                                            {
                                                                            }
                                                                            else
                                                                                continue;
                                                                        else
                                                                            continue;
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else if (p[pixel[14]] < c_b)
                                                    if (p[pixel[15]] < c_b)
                                                    {
                                                    }
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[7]] < c_b)
                                                if (p[pixel[8]] < c_b)
                                                {
                                                }
                                                else if (p[pixel[15]] < c_b)
                                                {
                                                }
                                                else
                                                    continue;
                                            else if (p[pixel[14]] < c_b)
                                                if (p[pixel[15]] < c_b)
                                                {
                                                }
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[13]] > cb)
                                            if (p[pixel[7]] > cb)
                                                if (p[pixel[8]] > cb)
                                                    if (p[pixel[9]] > cb)
                                                        if (p[pixel[10]] > cb)
                                                            if (p[pixel[11]] > cb)
                                                                if (p[pixel[12]] > cb)
                                                                    if (p[pixel[14]] > cb)
                                                                        if (p[pixel[15]] > cb)
                                                                        {
                                                                        }
                                                                        else
                                                                            continue;
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[13]] < c_b)
                                            if (p[pixel[14]] < c_b)
                                                if (p[pixel[15]] < c_b)
                                                {
                                                }
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if (p[pixel[12]] > cb)
                                        if (p[pixel[7]] > cb)
                                            if (p[pixel[8]] > cb)
                                                if (p[pixel[9]] > cb)
                                                    if (p[pixel[10]] > cb)
                                                        if (p[pixel[11]] > cb)
                                                            if (p[pixel[13]] > cb)
                                                                if (p[pixel[14]] > cb)
                                                                    if (p[pixel[6]] > cb)
                                                                    {
                                                                    }
                                                                    else if (p[pixel[15]] > cb)
                                                                    {
                                                                    }
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if (p[pixel[12]] < c_b)
                                        if (p[pixel[13]] < c_b)
                                            if (p[pixel[14]] < c_b)
                                                if (p[pixel[15]] < c_b)
                                                {
                                                }
                                                else if (p[pixel[6]] < c_b)
                                                    if (p[pixel[7]] < c_b)
                                                        if (p[pixel[8]] < c_b)
                                                            if (p[pixel[9]] < c_b)
                                                                if (p[pixel[10]] < c_b)
                                                                    if (p[pixel[11]] < c_b)
                                                                    {
                                                                    }
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if (p[pixel[11]] > cb)
                                    if (p[pixel[7]] > cb)
                                        if (p[pixel[8]] > cb)
                                            if (p[pixel[9]] > cb)
                                                if (p[pixel[10]] > cb)
                                                    if (p[pixel[12]] > cb)
                                                        if (p[pixel[13]] > cb)
                                                            if (p[pixel[6]] > cb)
                                                                if (p[pixel[5]] > cb)
                                                                {
                                                                }
                                                                else if (p[pixel[14]] > cb)
                                                                {
                                                                }
                                                                else
                                                                    continue;
                                                            else if (p[pixel[14]] > cb)
                                                                if (p[pixel[15]] > cb)
                                                                {
                                                                }
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if (p[pixel[11]] < c_b)
                                    if (p[pixel[12]] < c_b)
                                        if (p[pixel[13]] < c_b)
                                            if (p[pixel[14]] < c_b)
                                                if (p[pixel[15]] < c_b)
                                                {
                                                }
                                                else if (p[pixel[6]] < c_b)
                                                    if (p[pixel[7]] < c_b)
                                                        if (p[pixel[8]] < c_b)
                                                            if (p[pixel[9]] < c_b)
                                                                if (p[pixel[10]] < c_b)
                                                                {
                                                                }
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[5]] < c_b)
                                                if (p[pixel[6]] < c_b)
                                                    if (p[pixel[7]] < c_b)
                                                        if (p[pixel[8]] < c_b)
                                                            if (p[pixel[9]] < c_b)
                                                                if (p[pixel[10]] < c_b)
                                                                {
                                                                }
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else if (p[pixel[10]] > cb)
                                if (p[pixel[7]] > cb)
                                    if (p[pixel[8]] > cb)
                                        if (p[pixel[9]] > cb)
                                            if (p[pixel[11]] > cb)
                                                if (p[pixel[12]] > cb)
                                                    if (p[pixel[6]] > cb)
                                                        if (p[pixel[5]] > cb)
                                                            if (p[pixel[4]] > cb)
                                                            {
                                                            }
                                                            else if (p[pixel[13]] > cb)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else if (p[pixel[13]] > cb)
                                                            if (p[pixel[14]] > cb)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else if (p[pixel[13]] > cb)
                                                        if (p[pixel[14]] > cb)
                                                            if (p[pixel[15]] > cb)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else if (p[pixel[10]] < c_b)
                                if (p[pixel[11]] < c_b)
                                    if (p[pixel[12]] < c_b)
                                        if (p[pixel[13]] < c_b)
                                            if (p[pixel[14]] < c_b)
                                                if (p[pixel[15]] < c_b)
                                                {
                                                }
                                                else if (p[pixel[6]] < c_b)
                                                    if (p[pixel[7]] < c_b)
                                                        if (p[pixel[8]] < c_b)
                                                            if (p[pixel[9]] < c_b)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[5]] < c_b)
                                                if (p[pixel[6]] < c_b)
                                                    if (p[pixel[7]] < c_b)
                                                        if (p[pixel[8]] < c_b)
                                                            if (p[pixel[9]] < c_b)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[4]] < c_b)
                                            if (p[pixel[5]] < c_b)
                                                if (p[pixel[6]] < c_b)
                                                    if (p[pixel[7]] < c_b)
                                                        if (p[pixel[8]] < c_b)
                                                            if (p[pixel[9]] < c_b)
                                                            {
                                                            }
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else if (p[pixel[9]] > cb)
                            if (p[pixel[7]] > cb)
                                if (p[pixel[8]] > cb)
                                    if (p[pixel[10]] > cb)
                                        if (p[pixel[11]] > cb)
                                            if (p[pixel[6]] > cb)
                                                if (p[pixel[5]] > cb)
                                                    if (p[pixel[4]] > cb)
                                                        if (p[pixel[3]] > cb)
                                                        {
                                                        }
                                                        else if (p[pixel[12]] > cb)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else if (p[pixel[12]] > cb)
                                                        if (p[pixel[13]] > cb)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else if (p[pixel[12]] > cb)
                                                    if (p[pixel[13]] > cb)
                                                        if (p[pixel[14]] > cb)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[12]] > cb)
                                                if (p[pixel[13]] > cb)
                                                    if (p[pixel[14]] > cb)
                                                        if (p[pixel[15]] > cb)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else if (p[pixel[9]] < c_b)
                            if (p[pixel[10]] < c_b)
                                if (p[pixel[11]] < c_b)
                                    if (p[pixel[12]] < c_b)
                                        if (p[pixel[13]] < c_b)
                                            if (p[pixel[14]] < c_b)
                                                if (p[pixel[15]] < c_b)
                                                {
                                                }
                                                else if (p[pixel[6]] < c_b)
                                                    if (p[pixel[7]] < c_b)
                                                        if (p[pixel[8]] < c_b)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[5]] < c_b)
                                                if (p[pixel[6]] < c_b)
                                                    if (p[pixel[7]] < c_b)
                                                        if (p[pixel[8]] < c_b)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[4]] < c_b)
                                            if (p[pixel[5]] < c_b)
                                                if (p[pixel[6]] < c_b)
                                                    if (p[pixel[7]] < c_b)
                                                        if (p[pixel[8]] < c_b)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if (p[pixel[3]] < c_b)
                                        if (p[pixel[4]] < c_b)
                                            if (p[pixel[5]] < c_b)
                                                if (p[pixel[6]] < c_b)
                                                    if (p[pixel[7]] < c_b)
                                                        if (p[pixel[8]] < c_b)
                                                        {
                                                        }
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else if (p[pixel[8]] > cb)
                        if (p[pixel[7]] > cb)
                            if (p[pixel[9]] > cb)
                                if (p[pixel[10]] > cb)
                                    if (p[pixel[6]] > cb)
                                        if (p[pixel[5]] > cb)
                                            if (p[pixel[4]] > cb)
                                                if (p[pixel[3]] > cb)
                                                    if (p[pixel[2]] > cb)
                                                    {
                                                    }
                                                    else if (p[pixel[11]] > cb)
                                                    {
                                                    }
                                                    else
                                                        continue;
                                                else if (p[pixel[11]] > cb)
                                                    if (p[pixel[12]] > cb)
                                                    {
                                                    }
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[11]] > cb)
                                                if (p[pixel[12]] > cb)
                                                    if (p[pixel[13]] > cb)
                                                    {
                                                    }
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[11]] > cb)
                                            if (p[pixel[12]] > cb)
                                                if (p[pixel[13]] > cb)
                                                    if (p[pixel[14]] > cb)
                                                    {
                                                    }
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if (p[pixel[11]] > cb)
                                        if (p[pixel[12]] > cb)
                                            if (p[pixel[13]] > cb)
                                                if (p[pixel[14]] > cb)
                                                    if (p[pixel[15]] > cb)
                                                    {
                                                    }
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else if (p[pixel[8]] < c_b)
                        if (p[pixel[9]] < c_b)
                            if (p[pixel[10]] < c_b)
                                if (p[pixel[11]] < c_b)
                                    if (p[pixel[12]] < c_b)
                                        if (p[pixel[13]] < c_b)
                                            if (p[pixel[14]] < c_b)
                                                if (p[pixel[15]] < c_b)
                                                {
                                                }
                                                else if (p[pixel[6]] < c_b)
                                                    if (p[pixel[7]] < c_b)
                                                    {
                                                    }
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if (p[pixel[5]] < c_b)
                                                if (p[pixel[6]] < c_b)
                                                    if (p[pixel[7]] < c_b)
                                                    {
                                                    }
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[4]] < c_b)
                                            if (p[pixel[5]] < c_b)
                                                if (p[pixel[6]] < c_b)
                                                    if (p[pixel[7]] < c_b)
                                                    {
                                                    }
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if (p[pixel[3]] < c_b)
                                        if (p[pixel[4]] < c_b)
                                            if (p[pixel[5]] < c_b)
                                                if (p[pixel[6]] < c_b)
                                                    if (p[pixel[7]] < c_b)
                                                    {
                                                    }
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if (p[pixel[2]] < c_b)
                                    if (p[pixel[3]] < c_b)
                                        if (p[pixel[4]] < c_b)
                                            if (p[pixel[5]] < c_b)
                                                if (p[pixel[6]] < c_b)
                                                    if (p[pixel[7]] < c_b)
                                                    {
                                                    }
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else
                        continue;
                else if (p[pixel[7]] > cb)
                    if (p[pixel[8]] > cb)
                        if (p[pixel[9]] > cb)
                            if (p[pixel[6]] > cb)
                                if (p[pixel[5]] > cb)
                                    if (p[pixel[4]] > cb)
                                        if (p[pixel[3]] > cb)
                                            if (p[pixel[2]] > cb)
                                                if (p[pixel[1]] > cb)
                                                {
                                                }
                                                else if (p[pixel[10]] > cb)
                                                {
                                                }
                                                else
                                                    continue;
                                            else if (p[pixel[10]] > cb)
                                                if (p[pixel[11]] > cb)
                                                {
                                                }
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[10]] > cb)
                                            if (p[pixel[11]] > cb)
                                                if (p[pixel[12]] > cb)
                                                {
                                                }
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if (p[pixel[10]] > cb)
                                        if (p[pixel[11]] > cb)
                                            if (p[pixel[12]] > cb)
                                                if (p[pixel[13]] > cb)
                                                {
                                                }
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if (p[pixel[10]] > cb)
                                    if (p[pixel[11]] > cb)
                                        if (p[pixel[12]] > cb)
                                            if (p[pixel[13]] > cb)
                                                if (p[pixel[14]] > cb)
                                                {
                                                }
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else if (p[pixel[10]] > cb)
                                if (p[pixel[11]] > cb)
                                    if (p[pixel[12]] > cb)
                                        if (p[pixel[13]] > cb)
                                            if (p[pixel[14]] > cb)
                                                if (p[pixel[15]] > cb)
                                                {
                                                }
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else
                        continue;
                else if (p[pixel[7]] < c_b)
                    if (p[pixel[8]] < c_b)
                        if (p[pixel[9]] < c_b)
                            if (p[pixel[6]] < c_b)
                                if (p[pixel[5]] < c_b)
                                    if (p[pixel[4]] < c_b)
                                        if (p[pixel[3]] < c_b)
                                            if (p[pixel[2]] < c_b)
                                                if (p[pixel[1]] < c_b)
                                                {
                                                }
                                                else if (p[pixel[10]] < c_b)
                                                {
                                                }
                                                else
                                                    continue;
                                            else if (p[pixel[10]] < c_b)
                                                if (p[pixel[11]] < c_b)
                                                {
                                                }
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if (p[pixel[10]] < c_b)
                                            if (p[pixel[11]] < c_b)
                                                if (p[pixel[12]] < c_b)
                                                {
                                                }
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if (p[pixel[10]] < c_b)
                                        if (p[pixel[11]] < c_b)
                                            if (p[pixel[12]] < c_b)
                                                if (p[pixel[13]] < c_b)
                                                {
                                                }
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if (p[pixel[10]] < c_b)
                                    if (p[pixel[11]] < c_b)
                                        if (p[pixel[12]] < c_b)
                                            if (p[pixel[13]] < c_b)
                                                if (p[pixel[14]] < c_b)
                                                {
                                                }
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else if (p[pixel[10]] < c_b)
                                if (p[pixel[11]] < c_b)
                                    if (p[pixel[12]] < c_b)
                                        if (p[pixel[13]] < c_b)
                                            if (p[pixel[14]] < c_b)
                                                if (p[pixel[15]] < c_b)
                                                {
                                                }
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else
                        continue;
                else
                    continue;
                corners.push_back(Fastxy(static_cast<short>(x), static_cast<short>(y)));
            }
    }

} //namespace fast
