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

#include <mivins/img_align/frame_gpu.h>

#include <algorithm>
#include <stdexcept>
#include <fast/fast.h>
#include <mivins/utils/math_utils.h>
#include <mivins/utils/cv_utils.h>

/// @todo move to camera.h ?
#ifndef SVO_USE_VIKIT_CAMERA
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera-unified-projection.h>
#include <aslam/cameras/camera-omni.h>
#endif
#include <imp/bridge/opencv/cu_cv_bridge.hpp>

#include <mivins/common/logging.h>
#include <mivins/common/point.h>
#include <mivins/common/camera.h>

namespace mivins
{

    FrameGpu::FrameGpu(
        const CameraPtr &cam,
        const cv::Mat &img,
        const uint64_t timestamp_ns,
        size_t n_pyr_levels)
        : Frame(cam, img, timestamp_ns, n_pyr_levels)
    {
    }

    FrameGpu::FrameGpu(
        const int id,
        const uint64_t timestamp_ns,
        const CameraPtr &cam,
        const Transformation &T_world_cam)
        : Frame(id, timestamp_ns, cam, T_world_cam)
    {
    }

    FrameGpu::~FrameGpu()
    {
    }

    void FrameGpu::initGpuData(imp::cu::Matrix<FloatTypeGpu, 3, 4>::Ptr &cu_T_imu_cam,
                               imp::cu::Matrix<FloatTypeGpu, 3, 4>::Ptr &cu_T_cam_imu,
                               imp::cu::PinholeCamera::Ptr &cu_camera)
    {
        // Copy image pyramid to GPU
        // TODO: USE imp::ImagePyramid
        cu_img_pyramid_copy_.resize(img_pyr_.size());
        for (int i = 0; i < static_cast<int>(img_pyr_.size()); ++i)
        {
            cu_img_pyramid_copy_.at(i) = std::make_shared<imp::cu::ImageGpu8uC1>(
                imp::cu::ImageGpu8uC1(imp::ImageCv8uC1(img_pyr_.at(i))));
        }

        // Initialize transformations
        cu_T_imu_cam_ = cu_T_imu_cam;
        cu_T_cam_imu_ = cu_T_cam_imu;
        cu_camera_ = cu_camera;
    }

} // namespace mivins
