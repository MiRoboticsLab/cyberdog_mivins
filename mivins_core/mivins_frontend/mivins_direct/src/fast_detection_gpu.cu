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
#include "fast/fast.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <iostream>

#include <mivins/direct/fast_detection_gpu.h>
//#include <mivins/direct/feature_detector_utilities.h>
//#include <mivins/direct/feature_detector_types.h>
//#include <mivins/direct/feature_detector.h>

__device__ int MinMaxScore(int *scores, const int threshold) 
{
    int score_max = 0;

    for (int i = 0; i < 16; i++)
    {
        if (scores[(i + 0) & 0xf] > threshold &&
            scores[(i + 1) & 0xf] > threshold &&
            scores[(i + 2) & 0xf] > threshold &&
            scores[(i + 3) & 0xf] > threshold &&
            scores[(i + 4) & 0xf] > threshold &&
            scores[(i + 5) & 0xf] > threshold &&
            scores[(i + 6) & 0xf] > threshold &&
            scores[(i + 7) & 0xf] > threshold &&
            scores[(i + 8) & 0xf] > threshold &&
            scores[(i + 9) & 0xf] > threshold)
        {
            int score_min_tmp_0 = min(scores[(i + 0) & 0xf], scores[(i + 1) & 0xf]);
            int score_min_tmp_1 = min(scores[(i + 2) & 0xf], scores[(i + 3) & 0xf]);
            int score_min_tmp_2 = min(scores[(i + 4) & 0xf], scores[(i + 5) & 0xf]);
            int score_min_tmp_3 = min(scores[(i + 6) & 0xf], scores[(i + 7) & 0xf]);
            int score_min_tmp_4 = min(scores[(i + 8) & 0xf], scores[(i + 9) & 0xf]);
            score_min_tmp_0 = min(score_min_tmp_0, score_min_tmp_1);
            score_min_tmp_2 = min(score_min_tmp_2, score_min_tmp_3);
            score_min_tmp_0 = min(score_min_tmp_0, score_min_tmp_2);
            score_min_tmp_0 = min(score_min_tmp_0, score_min_tmp_4);
            score_max = max(score_max, score_min_tmp_0);
        }
        else if (scores[(i + 0) & 0xf] < -threshold && 
                 scores[(i + 1) & 0xf] < -threshold && 
                 scores[(i + 2) & 0xf] < -threshold && 
                 scores[(i + 3) & 0xf] < -threshold && 
                 scores[(i + 4) & 0xf] < -threshold && 
                 scores[(i + 5) & 0xf] < -threshold && 
                 scores[(i + 6) & 0xf] < -threshold && 
                 scores[(i + 7) & 0xf] < -threshold && 
                 scores[(i + 8) & 0xf] < -threshold && 
                 scores[(i + 9) & 0xf] < -threshold )
        {
            int score_min_tmp_0 = max(scores[(i + 0) & 0xf], scores[(i + 1) & 0xf]);
            int score_min_tmp_1 = max(scores[(i + 2) & 0xf], scores[(i + 3) & 0xf]);
            int score_min_tmp_2 = max(scores[(i + 4) & 0xf], scores[(i + 5) & 0xf]);
            int score_min_tmp_3 = max(scores[(i + 6) & 0xf], scores[(i + 7) & 0xf]);
            int score_min_tmp_4 = max(scores[(i + 8) & 0xf], scores[(i + 9) & 0xf]);
            score_min_tmp_0 = max(score_min_tmp_0, score_min_tmp_1);
            score_min_tmp_2 = max(score_min_tmp_2, score_min_tmp_3);
            score_min_tmp_0 = max(score_min_tmp_0, score_min_tmp_2);
            score_min_tmp_0 = max(score_min_tmp_0, score_min_tmp_4);
            score_max = max(score_max, abs(score_min_tmp_0));
        }
    }
    return score_max;
}

__device__ int Comparator(uint8_t *input, const int *circle, const int threshold, int index) 
{
    uint8_t pixel = input[index];
    int scores[16] = {0};

    /// iterate over whole circle
    for (size_t i = 0; i < 16; i++) // 16 + 10  --circle_size=16 --consecutive_point_size=10
    {
        scores[i] = pixel - input[index + circle[i]]; // i % circle_size

    }

    return MinMaxScore(scores, threshold + 1) - 1;
}

__global__ void FastKernel(uint8_t *src, int32_t *dst, int32_t *mlock, 
                           const int width, const int height, const int threshold, 
                           const int border, const int scale, const int cell_size, 
                           const int grid_size_x, const int level)
{
    int block_size = blockDim.x;
    int local_size = blockDim.x + 2;
    extern __shared__ int local_score[]; // block_size * block_size

    // get 1d coordinates and cutout borders
    const int pixel[16] = {
        0 + width * 3,
        1 + width * 3,
        2 + width * 2,
        3 + width * 1,
        3 + width * 0,
        3 + width * -1,
        2 + width * -2,
        1 + width * -3,
        0 + width * -3,
        -1 + width * -3,
        -2 + width * -2,
        -3 + width * -1,
        -3 + width * 0,
        -3 + width * 1,
        -2 + width * 2,
        -1 + width * 3,
    };

    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int idy = blockIdx.y * blockDim.y + threadIdx.y;

    int block_id = floor((float)idy * scale / cell_size) * grid_size_x + floor((float)idx * scale / cell_size);

    int start_index = threadIdx.y * blockDim.x + threadIdx.x;
    for (int index_local = start_index; index_local < local_size * local_size; index_local += block_size * block_size)
    {
        int y = blockIdx.y * blockDim.y - 1 + index_local / local_size;
        int x = blockIdx.x * blockDim.x - 1 + index_local % local_size;
        int index_global = y * width + x;

        if (x < 3 || y < 3 || x >= (width - 3) || y >= (height - 3))
        {
            local_score[index_local] = 0;
            continue;
        }
        local_score[index_local] = Comparator(src, pixel, threshold, index_global);
    }

    __syncthreads();
    int score_center_index = (threadIdx.y + 1) * local_size + threadIdx.x + 1;
    int score_center = local_score[score_center_index];

    bool blocked = true;
    if (score_center > local_score[score_center_index - local_size - 1] &&
        score_center > local_score[score_center_index - local_size - 0] &&
        score_center > local_score[score_center_index - local_size + 1] &&
        score_center > local_score[score_center_index - 1] &&
        score_center > local_score[score_center_index + 1] &&
        score_center > local_score[score_center_index + local_size - 1] &&
        score_center > local_score[score_center_index + local_size - 0] &&
        score_center > local_score[score_center_index + local_size + 1])
    {
        while (blocked)
        {
            if (0 == atomicCAS(&mlock[block_id], 0, 1))
            {
                if (idx >= border && idy >= border && idx < (width - border) && idy < (height - border))
                {
                    if ((score_center > *(dst + 4 * block_id + 3)) || 
                        ((score_center == *(dst + 4 * block_id + 3)) && 
                         (level == *(dst + 4 * block_id + 2)) &&
                         ((idy * scale < *(dst + 4 * block_id + 1)) || 
                          ((idy * scale == *(dst + 4 * block_id + 1)) && 
                           (idx * scale < *(dst + 4 * block_id))))))
                    {
                        *(dst + 4 * block_id    ) = idx * scale;
                        *(dst + 4 * block_id + 1) = idy * scale;
                        *(dst + 4 * block_id + 2) = level;
                        *(dst + 4 * block_id + 3) = score_center;
                    }
                }
                __threadfence();
                atomicExch(&mlock[block_id], 0);
                blocked = false;
            }
        }
    }
}

namespace mivins
{
    namespace feature_detector_utils
    {
        void FastDetectorGpu(
            const ImgPyramid &img_pyr,
            const int threshold,
            const int border,
            const size_t min_level,
            const size_t max_level,
            Corners &corners,
            OccupandyGrid2D& grid)
        {
            int block_size = grid.cell_size;
            int cell_size = grid.cell_size;
            int grid_size_x = grid.n_cols;
            int grid_size_y = grid.n_rows;
            int grid_size   = grid_size_x * grid_size_y;

            dim3 blocks(block_size, block_size);
            dim3 grids(std::ceil((float)img_pyr[0].cols / block_size), std::ceil((float)img_pyr[0].cols / block_size));
            uint8_t *p_mem_src = NULL;
            int32_t *p_mem_dst = NULL;
            int32_t *p_mlock = NULL;
            cudaMallocManaged(&p_mem_src, img_pyr[0].total());
            cudaMallocManaged(&p_mem_dst, grid_size * 16);
            cudaMallocManaged(&p_mlock, grid_size * 4);
            cudaMemset(p_mem_dst, 0, grid_size * 16);
            cudaMemset(p_mlock, 0, grid_size * 4);

            for (int level = min_level; level <= max_level; level++)
            {
                const int scale  = (1 << level);
                const int width  = img_pyr[level].cols;
                const int height = img_pyr[level].rows;

                // cudaMemcpy(p_mem_src, img_pyr[level].data, img_pyr[level].total(), cudaMemcpyHostToDevice);
                cudaStreamAttachMemAsync(NULL, p_mem_src, 0, cudaMemAttachHost);
                memcpy(p_mem_src, (void *)img_pyr[level].data, img_pyr[level].total());
                cudaStreamAttachMemAsync(NULL, p_mem_src, 0, cudaMemAttachGlobal);

                FastKernel<<<grids, blocks, (block_size + 2) * (block_size + 2) * 4>>>(
                    p_mem_src, p_mem_dst, p_mlock, width, height, 
                    threshold, border, scale, cell_size, grid_size_x, level);
                cudaDeviceSynchronize();
            }

            cudaStreamAttachMemAsync(NULL, p_mem_dst, 0, cudaMemAttachHost);
            for (int i = 0; i < grid_size; i++)
            {
                if (grid.occupancy_.at(i))
                    continue;

                if (p_mem_dst[4 * i + 3] > 0)
                {
                    corners.at(i).x = p_mem_dst[4 * i + 0];
                    corners.at(i).y = p_mem_dst[4 * i + 1];
                    corners.at(i).level = p_mem_dst[4 * i + 2];
                    corners.at(i).score = p_mem_dst[4 * i + 3];
                }
            }
            cudaFree(p_mem_src);
            cudaFree(p_mem_dst);
            cudaFree(p_mlock);
        }
    } //feature_detector_utils
} //mivins