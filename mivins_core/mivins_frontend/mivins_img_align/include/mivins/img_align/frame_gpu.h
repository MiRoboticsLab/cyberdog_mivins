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

#include <mutex>
#include <unordered_map>
#include <opencv2/core/core.hpp>
#include <mivins/utils/math_utils.h>
#include <imp/cu_imgproc/image_pyramid.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_matrix.cuh>
#include <imp/cu_core/cu_pinhole_camera.cuh>
#include <mivins/common/frame.h>
#include <mivins/common/types.h>
#include <mivins/common/transformation.h>
#include <mivins/common/camera_fwd.h>
#include <mivins/common/feature_wrapper.h>
#include <mivins/common/seed_wrapper.h>
#include <mivins/img_align/sparse_img_align_device_utils.cuh> //!< for FloatTypeGpu

namespace mivins
{

    /**
 * @brief The FrameGpu class adds GPU data members to the frame base class.
 */
    class FrameGpu : public Frame
    {
    public:
        typedef std::shared_ptr<FrameGpu> Ptr;

        std::vector<imp::cu::ImageGpu8uC1::Ptr> cu_img_pyramid_copy_; //!< Image Pyramid
        // TODO: USE imp::ImagePyramid
        //std::vector<imp::ImagePyramid8uC1::Ptr> cu_ref_pyramids_device_;
        imp::cu::Matrix<FloatTypeGpu, 3, 4>::Ptr cu_T_imu_cam_;
        imp::cu::Matrix<FloatTypeGpu, 3, 4>::Ptr cu_T_cam_imu_;
        imp::cu::PinholeCamera::Ptr cu_camera_;

        /// Default Constructor
        FrameGpu(const CameraPtr &cam,
                 const cv::Mat &img,
                 const uint64_t timestamp_ns,
                 const size_t n_pyr_levels);

        /// Constructor without image. Just for testing!
        FrameGpu(
            const int id,
            const uint64_t timestamp_ns,
            const CameraPtr &cam,
            const Transformation &T_world_cam);

        /// Empty constructor. Just for testing!
        FrameGpu() {}

        /// Destructor
        virtual ~FrameGpu();

        // no copy
        FrameGpu(const FrameGpu &) = delete;
        FrameGpu &operator=(const FrameGpu &) = delete;

        /// Initialize new frame and create image pyramid.
        void initGpuData(imp::cu::Matrix<FloatTypeGpu, 3, 4>::Ptr &cu_T_imu_cam,
                         imp::cu::Matrix<FloatTypeGpu, 3, 4>::Ptr &cu_T_cam_imu,
                         imp::cu::PinholeCamera::Ptr &cu_camera);
    };

} // namespace mivins
