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

#include "cuda_runtime.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <iostream>
#include <mivins/direct/fast_detection_gpu.h>

#define M_PI_LOCAL 3.14159265358979323846

struct Corner
{
  int x;        ///< x-coordinate of corner in the image.
  int y;        ///< y-coordinate of corner in the image.
  int level;    ///< pyramid level of the corner.
  float score;  ///< shi-tomasi score of the corner.
  float angle;  ///< for gradient-features: dominant gradient angle.
};

__device__ float getAngleAtPixelUsingHistogramCuda(const unsigned char *src_img, int idx, int idy, int halfpatch_size, int width, int height)
{
    int n_bins = 36;
    double hist[36] = {0.0};

    // angleHistogram
    constexpr double pi2 = 2.0 * M_PI_LOCAL;
    for (int dy = -halfpatch_size; dy <= halfpatch_size; ++dy)
    {
        for (int dx = -halfpatch_size; dx <= halfpatch_size; ++dx)
        {
            // gradientAndMagnitudeAtPixel
            int index_x = idx + dx;
            int index_y = idy + dy; 
            if (index_y < 0 || index_y >= height - 1 || index_x < 0 || index_x >= width - 1)
                continue;
            int ddx = src_img[index_y * width + index_x + 1] - src_img[index_y * width + index_x - 1];
            int ddy = src_img[(index_y + 1) * width + index_x] - src_img[(index_y - 1) * width + index_x];
            float mag = sqrt((float)(ddx * ddx + ddy * ddy));
            float angle = atan2((float)ddy, (float)ddx); 

            size_t bin = round( n_bins * (angle + M_PI_LOCAL) / pi2);
            bin = (bin < n_bins) ? bin : 0u;
            hist[bin] += mag;
        }
    }

    // smoothOrientationHistogram
    double prev = hist[n_bins - 1], h0 = hist[0];
    for (int i = 0; i < n_bins; ++i)
    {
        double temp = hist[i];
        hist[i] = 0.25 * prev + 0.5 * hist[i] + 0.25 * ( (n_bins == i + 1 ) ? h0 : hist[i + 1]);
        prev = temp;
    }

    // dgetDominantAngle
    double max_angle = hist[0];
    int max_bin = 0;
    for (int i = 1; i < n_bins; i++)
    {
        if (hist[i] > max_angle)
        {
            max_angle = hist[i];
            max_bin = i;
        }
    }

    return max_bin * 2.0 * M_PI_LOCAL / n_bins;
}

__global__ void Gaussian3x3Kernel(const unsigned char *src_data, unsigned char *gauss_data, const int width, const int height)
{
    const int id_x = blockIdx.x * blockDim.x + threadIdx.x;
    const int id_y = blockIdx.y * blockDim.y + threadIdx.y;

    if (id_x >= width - 1 || id_y >= height - 1 || id_x == 0 || id_y == 0)
        return ;

    const int index = id_x + id_y * width;
    const int t_index = index - width;
    const int b_index = index + width;

    unsigned char ret = (src_data[t_index - 1] + src_data[t_index + 1] + src_data[b_index - 1] + src_data[b_index + 1] +
                      ((src_data[t_index] + src_data[index - 1] + src_data[index + 1] + src_data[b_index]) << 1) +
                       (src_data[index] << 2)) >> 4;
    gauss_data[index] = ret;
}

__global__ void ScharrForScore(const unsigned char *gauss_data, float *score_data,
                               const float threshold, const int border, const int width, const int height)
{
    const int id_x = blockIdx.x * blockDim.x + threadIdx.x;
    const int id_y = blockIdx.y * blockDim.y + threadIdx.y;

    if (id_x >= width - border || id_y >= height - border || id_x < border || id_y < border)
        return;

    const int index = id_x + id_y * width;
    const int t_index = index - width;
    const int b_index = index + width;

    short dx = (short)((gauss_data[t_index + 1] + gauss_data[b_index + 1] - gauss_data[t_index - 1] - gauss_data[b_index - 1]) * 3 +
                       (gauss_data[index + 1] - gauss_data[index - 1]) * 10);
    short dy = (short)((gauss_data[b_index - 1] + gauss_data[b_index + 1] - gauss_data[t_index - 1] - gauss_data[t_index + 1]) * 3 +
                       (gauss_data[b_index] - gauss_data[t_index]) * 10);

    float mag = sqrt((float)(dx * dx + dy * dy));
    float score = 0.0f;
    if (mag > threshold)
    {
        score = mag;
    }
    score_data[index] = score;
}

__global__ void NonmaxSuppression(const unsigned char *src_img, float *score_data, void *p_corner, unsigned char *occupancy, int *mlock,
                                  const int border, const int scale, const int level, const int cell_size, const int cell_cols,
                                  const int width, const int height)
{
    const int id_x = blockIdx.x * blockDim.x + threadIdx.x;
    const int id_y = blockIdx.y * blockDim.y + threadIdx.y;

    if (id_x >= width - border || id_y >= height - border || id_x < border || id_y < border)
        return;

    Corner *p = (Corner *)p_corner;
    int k = (int)( floor(id_y * scale / (float)cell_size) * cell_cols + floor(id_x * scale / (float)cell_size) );
    if (occupancy[k] == 1)
        return;
    
    const int index = id_x + id_y * width;
    const int t_index = index - width;
    const int b_index = index + width;

    if((score_data[index] > 0.0f) 
    && (score_data[index] >= score_data[t_index - 1])
    && (score_data[index] >= score_data[t_index]) 
    && (score_data[index] > score_data[t_index + 1])
    && (score_data[index] >= score_data[index - 1])
    && (score_data[index] > score_data[index + 1])
    && (score_data[index] >= score_data[b_index - 1])
    && (score_data[index] > score_data[b_index])
    && (score_data[index] > score_data[b_index + 1]) )
    {
        bool blocked = true;
        while (blocked)
        {
            if (0 == atomicCAS(&mlock[k], 0, 1))
            {
                if (score_data[index] > p[k].score)
                {
                    p[k].x = id_x * scale;
                    p[k].y = id_y * scale;
                    p[k].level = level - 1;
                    p[k].score = score_data[index];
                    //p[k].angle = getAngleAtPixelUsingHistogramCuda(src_img, id_x, id_y, 4, width, height);
                }
                __threadfence();
                atomicExch(&mlock[k], 0);
                blocked = false;
            }
        }
    }
}

__global__ void GetAngleByCorner(const unsigned char *src_img, void *p_corner, 
                                 const int threshold, const int scale, const int corner_size, const int width, const int height)
{
    const int id_x = blockIdx.x * blockDim.x + threadIdx.x;

    if (id_x >= corner_size)
        return;

    Corner *p = (Corner *)p_corner;

    if (p[id_x].score > (float)threshold)
    {
        p[id_x].angle = getAngleAtPixelUsingHistogramCuda(src_img, (p[id_x].x / scale), (p[id_x].y / scale), 4, width, height);
    }
}

namespace mivins
{
    namespace feature_detector_utils
    {
        void EdgeletDetectorV2Gpu(
            const ImgPyramid &img_pyr,
            const int threshold,
            const int border,
            const size_t min_level,
            const size_t max_level,
            Corners &corners,
            OccupandyGrid2D& grid)
        {
            constexpr int level = 1;
            constexpr int scale = (1 << level);
            int cell_size = grid.cell_size;

            void *src_img = (void *)img_pyr[level].data;
            int width = img_pyr[level].cols;
            int height = img_pyr[level].rows;
            size_t u8_data_size = width * height * sizeof(unsigned char);
            size_t corner_size = corners.size() * sizeof(Corner);

            dim3 block1(32);
            dim3 grid1((corners.size() - 1) / block1.x + 1);
            dim3 block2(cell_size, cell_size);
            dim3 grid2((width - 1) / block2.x + 1, (height - 1) / block2.y + 1);

            unsigned char *src_data = NULL;
            unsigned char *gauss_data = NULL; 
            unsigned char *p_corner = NULL;
            unsigned char *p_occupancy = NULL;
            float *score_data = NULL;
            int32_t *mlock = NULL;

            cudaMallocManaged(&src_data, u8_data_size, cudaMemAttachHost);
            cudaMalloc(&gauss_data, u8_data_size);

            cudaMalloc(&score_data, width * height * sizeof(float));
            cudaMalloc(&mlock, corners.size() * sizeof(int32_t));
            cudaMallocManaged(&p_corner, corner_size, cudaMemAttachHost);
            cudaMallocManaged(&p_occupancy, grid.occupancy_.size() * sizeof(unsigned char), cudaMemAttachHost);

            memcpy((void *)src_data, src_img, u8_data_size);
            cudaStreamAttachMemAsync(NULL, src_data, 0, cudaMemAttachGlobal);

            Gaussian3x3Kernel<<<grid2, block2>>>(src_data, gauss_data, width, height);

            ScharrForScore<<<grid2, block2>>>(gauss_data, score_data, threshold, border, width, height);

            memcpy((void *)p_corner, corners.data(), corner_size);
            cudaStreamAttachMemAsync(NULL, p_corner, 0, cudaMemAttachGlobal);

            for (int i = 0; i < grid.occupancy_.size(); i++)
            {
                if(grid.occupancy_.at(i))
                    p_occupancy[i] = 1;
                else
                    p_occupancy[i] = 0;
            }

            cudaStreamAttachMemAsync(NULL, p_occupancy, 0, cudaMemAttachGlobal);
            cudaMemset(mlock, 0, corners.size() * sizeof(int32_t));

            NonmaxSuppression<<<grid2, block2>>>(src_data, score_data, (void *)p_corner, p_occupancy, mlock,
                                                 border, scale, level, grid.cell_size, grid.n_cols,
                                                 width, height);

            GetAngleByCorner<<<grid1, block1>>>(src_data, p_corner, threshold, scale, corners.size(), width, height);

            cudaStreamAttachMemAsync(NULL, p_corner, 0, cudaMemAttachHost);
            cudaStreamSynchronize(NULL);
            memcpy(corners.data(), p_corner, corner_size);

            cudaFree(p_occupancy);
            cudaFree(mlock);
            cudaFree(p_corner);
            cudaFree(score_data);
            cudaFree(gauss_data);
            cudaFree(src_data);
        }
    } //feature_detector_utils
} //feature_detector_utils