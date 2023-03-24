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

#include <mivins/common/types.h>

#if __SSSE3__
#include <tmmintrin.h>
#endif
#ifdef __ARM_NEON__
#include <arm_neon.h>
#endif

namespace mivins
{
    namespace patch_score
    {

#if __SSSE3__
        // Horizontal sum of uint16s stored in an XMM register
        inline int SumXMM_16(__m128i &target)
        {
            unsigned short int sums_store[8];
            _mm_storeu_si128((__m128i *)sums_store, target);
            return sums_store[0] + sums_store[1] + sums_store[2] + sums_store[3] +
                   sums_store[4] + sums_store[5] + sums_store[6] + sums_store[7];
        }
        // Horizontal sum of uint32s stored in an XMM register
        inline int SumXMM_32(__m128i &target)
        {
            unsigned int sums_store[4];
            _mm_storeu_si128((__m128i *)sums_store, target);
            return sums_store[0] + sums_store[1] + sums_store[2] + sums_store[3];
        }
#endif

        /// Zero Mean Sum of Squared Differences Cost
        template <int HALF_PATCH_SIZE>
        class ZMSSD
        {
        public:
            static const int c_patch_size = 2 * HALF_PATCH_SIZE;
            static const int c_patch_area = c_patch_size * c_patch_size;
            static const int c_threshold = 2000 * c_patch_area;
            uint8_t *m_ref_patch;
            int m_sumA, m_sumAA;

            ZMSSD(uint8_t *ref_patch) : m_ref_patch(ref_patch)
            {
#ifdef __ARM_NEON__
                if (c_patch_size == 0)
                {
                    uint16x8_t tpl_x8;
                    uint16x8_t sumAx8 = vdupq_n_u16(0);
                    uint32x4_t sumAAx4 = vdupq_n_u32(0);
                    for (int i = 0; i < 64; i += 8)
                    {
                        tpl_x8 = vmovl_u8(vld1_u8(&m_ref_patch[i]));
                        sumAx8 = vaddq_u16(sumAx8, tpl_x8);
                        sumAAx4 = vmlal_u16(sumAAx4, vget_low_u16(tpl_x8), vget_low_u16(tpl_x8));
                        sumAAx4 = vmlal_u16(sumAAx4, vget_high_u16(tpl_x8), vget_high_u16(tpl_x8));
                    }
                    uint16x4_t sumAx4 = vpadd_u16(vget_low_u16(sumAx8), vget_high_u16(sumAx8));
                    m_sumA = vget_lane_u16(sumAx4, 0) + vget_lane_u16(sumAx4, 1) + vget_lane_u16(sumAx4, 2) + vget_lane_u16(sumAx4, 3);

                    uint32x2_t sumAAx2 = vpadd_u32(vget_low_u32(sumAAx4), vget_high_u32(sumAAx4));
                    m_sumAA = vget_lane_u32(sumAAx2, 0) + vget_lane_u32(sumAAx2, 1);
                }
                else
#endif
                {
                    uint32_t sumA_uint = 0, sumAA_uint = 0;
                    for (int r = 0; r < c_patch_area; r++)
                    {
                        uint8_t n = m_ref_patch[r];
                        sumA_uint += n;
                        sumAA_uint += n * n;
                    }
                    m_sumA = sumA_uint;
                    m_sumAA = sumAA_uint;
                }
            }

            static int threshold() { return c_threshold; }

            int computeScore(uint8_t *cur_patch) const
            {
                uint32_t sumB_uint = 0;
                uint32_t sumBB_uint = 0;
                uint32_t sumAB_uint = 0;
                for (int r = 0; r < c_patch_area; r++)
                {
                    const uint8_t cur_pixel = cur_patch[r];
                    sumB_uint += cur_pixel;
                    sumBB_uint += cur_pixel * cur_pixel;
                    sumAB_uint += cur_pixel * m_ref_patch[r];
                }
                const int sumB = sumB_uint;
                const int sumBB = sumBB_uint;
                const int sumAB = sumAB_uint;
                return m_sumAA - 2 * sumAB + sumBB - (m_sumA * m_sumA - 2 * m_sumA * sumB + sumB * sumB) / c_patch_area;
            }

            int computeScore(uint8_t *cur_patch, int stride) const
            {
                int sumB, sumBB, sumAB;
#ifdef __SSSE3__
                if (c_patch_size == 8)
                {
                    // From PTAM-GPL, Copyright 2008 Isis Innovation Limited
                    __m128i xImageAsEightBytes;
                    __m128i xImageAsWords;
                    __m128i xTemplateAsEightBytes;
                    __m128i xTemplateAsWords;
                    __m128i xZero;
                    __m128i xImageSums;   // These sums are 8xuint16
                    __m128i xImageSqSums; // These sums are 4xint32
                    __m128i xCrossSums;   // These sums are 4xint32
                    __m128i xProduct;

                    xImageSums = _mm_setzero_si128();
                    xImageSqSums = _mm_setzero_si128();
                    xCrossSums = _mm_setzero_si128();
                    xZero = _mm_setzero_si128();

                    uint8_t *imagepointer = cur_patch;
                    uint8_t *templatepointer = m_ref_patch;
                    long unsigned int cur_stride = stride;

                    xImageAsEightBytes = _mm_loadl_epi64((__m128i *)imagepointer);
                    imagepointer += cur_stride;
                    xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
                    xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
                    xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
                    xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
                    xTemplateAsEightBytes = _mm_load_si128((__m128i *)templatepointer);
                    templatepointer += 16;
                    xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes, xZero);
                    xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
                    xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

                    xImageAsEightBytes = _mm_loadl_epi64((__m128i *)imagepointer);
                    imagepointer += cur_stride;
                    xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
                    xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
                    xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
                    xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
                    xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes, xZero);
                    xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
                    xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

                    xImageAsEightBytes = _mm_loadl_epi64((__m128i *)imagepointer);
                    imagepointer += cur_stride;
                    xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
                    xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
                    xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
                    xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
                    xTemplateAsEightBytes = _mm_load_si128((__m128i *)templatepointer);
                    templatepointer += 16;
                    xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes, xZero);
                    xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
                    xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
                    xImageAsEightBytes = _mm_loadl_epi64((__m128i *)imagepointer);
                    imagepointer += cur_stride;
                    xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
                    xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
                    xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
                    xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
                    xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes, xZero);
                    xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
                    xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

                    xImageAsEightBytes = _mm_loadl_epi64((__m128i *)imagepointer);
                    imagepointer += cur_stride;
                    xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
                    xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
                    xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
                    xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
                    xTemplateAsEightBytes = _mm_load_si128((__m128i *)templatepointer);
                    templatepointer += 16;
                    xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes, xZero);
                    xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
                    xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
                    xImageAsEightBytes = _mm_loadl_epi64((__m128i *)imagepointer);
                    imagepointer += cur_stride;
                    xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
                    xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
                    xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
                    xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
                    xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes, xZero);
                    xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
                    xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

                    xImageAsEightBytes = _mm_loadl_epi64((__m128i *)imagepointer);
                    imagepointer += cur_stride;
                    xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
                    xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
                    xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
                    xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
                    xTemplateAsEightBytes = _mm_load_si128((__m128i *)templatepointer);
                    templatepointer += 16;
                    xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes, xZero);
                    xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
                    xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
                    xImageAsEightBytes = _mm_loadl_epi64((__m128i *)imagepointer);
                    xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
                    xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
                    xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
                    xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
                    xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes, xZero);
                    xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
                    xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

                    sumB = SumXMM_16(xImageSums);
                    sumAB = SumXMM_32(xCrossSums);
                    sumBB = SumXMM_32(xImageSqSums);
                }
                else
#endif
#ifdef __ARM_NEON__
                    if (c_patch_size == 8)
                {
                    uint16x8_t img_x8, tpl_x8;
                    uint16x8_t sumBx8 = vdupq_n_u16(0);
                    uint32x4_t sumBBx4 = vdupq_n_u32(0);
                    uint32x4_t sumABx4 = vdupq_n_u32(0);
                    uint8_t *cur_ptr = cur_patch;
                    uint8_t *ref_ptr = m_ref_patch;
                    for (int i = 0; i < 64; i += 8, cur_ptr += stride, ref_ptr += 8)
                    {
                        // load first row of image and tpl
                        img_x8 = vmovl_u8(vld1_u8(cur_ptr));
                        tpl_x8 = vmovl_u8(vld1_u8(ref_ptr));
                        // sum img
                        sumBx8 = vaddq_u16(sumBx8, img_x8);
                        // multiply and sum
                        sumBBx4 = vmlal_u16(sumBBx4, vget_low_u16(img_x8), vget_low_u16(img_x8));
                        sumBBx4 = vmlal_u16(sumBBx4, vget_high_u16(img_x8), vget_high_u16(img_x8));
                        // multiply and sum
                        sumABx4 = vmlal_u16(sumABx4, vget_low_u16(tpl_x8), vget_low_u16(img_x8));
                        sumABx4 = vmlal_u16(sumABx4, vget_high_u16(tpl_x8), vget_high_u16(img_x8));
                    }
                    uint16x4_t sumBx4 = vpadd_u16(vget_low_u16(sumBx8), vget_high_u16(sumBx8));
                    sumB = vget_lane_u16(sumBx4, 0) + vget_lane_u16(sumBx4, 1) + vget_lane_u16(sumBx4, 2) + vget_lane_u16(sumBx4, 3);

                    uint32x2_t sumBBx2 = vpadd_u32(vget_low_u32(sumBBx4), vget_high_u32(sumBBx4));
                    sumBB = vget_lane_u32(sumBBx2, 0) + vget_lane_u32(sumBBx2, 1);

                    uint32x2_t sumABx2 = vpadd_u32(vget_low_u32(sumABx4), vget_high_u32(sumABx4));
                    sumAB = vget_lane_u32(sumABx2, 0) + vget_lane_u32(sumABx2, 1);
                }
                else
#endif
                {
                    uint32_t sumB_uint = 0;
                    uint32_t sumBB_uint = 0;
                    uint32_t sumAB_uint = 0;
                    for (int y = 0, r = 0; y < c_patch_size; ++y)
                    {
                        uint8_t *cur_patch_ptr = cur_patch + y * stride;
                        for (int x = 0; x < c_patch_size; ++x, ++r)
                        {
                            const uint8_t cur_px = cur_patch_ptr[x];
                            sumB_uint += cur_px;
                            sumBB_uint += cur_px * cur_px;
                            sumAB_uint += cur_px * m_ref_patch[r];
                        }
                    }
                    sumB = sumB_uint;
                    sumBB = sumBB_uint;
                    sumAB = sumAB_uint;
                }
                return m_sumAA - 2 * sumAB + sumBB - (m_sumA * m_sumA - 2 * m_sumA * sumB + sumB * sumB) / c_patch_area;
            }
        };

    } // namespace patch_score
} // namespace mivins
