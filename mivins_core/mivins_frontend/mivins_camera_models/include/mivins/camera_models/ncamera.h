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

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <mivins/utils/transformation_utils.h>
#include <opencv2/opencv.hpp>

namespace vk
{
    namespace cameras
    {

        class CameraGeometryBase;
        using Camera = CameraGeometryBase;
        using Transformation = mivins::Transformation;
        using TransformationVector = mivins::TransformationVector;
        using Quaternion = mivins::Quaternion;

        class NCamera
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            typedef std::shared_ptr<NCamera> Ptr;
            typedef std::shared_ptr<const NCamera> ConstPtr;

        protected:
            NCamera() = default;

        public:
            NCamera(
                const TransformationVector &T_C_B,
                const std::vector<std::shared_ptr<Camera>> &cameras,
                const std::string &label);

            ~NCamera() {}

            NCamera(const NCamera &) = delete;
            void operator=(const NCamera &) = delete;

            /// Load a camera rig form a yaml file. Returns a nullptr if the loading fails.
            static std::shared_ptr<NCamera> loadFromYaml(const std::string &yaml_file);

            /// Get the number of cameras.
            inline size_t getNumCameras() const { return cameras_.size(); }

            /// Get the pose of body frame with respect to the camera i.
            const Transformation &get_T_C_B(size_t camera_index) const;

            /// Get all transformations.
            const TransformationVector &getTransformationVector() const;

            /// Get the geometry object for camera i.
            const Camera &getCamera(size_t camera_index) const;

            /// Get the geometry object for camera i.
            std::shared_ptr<Camera> getCameraShared(size_t camera_index);

            /// Get the geometry object for camera i.
            std::shared_ptr<const Camera> getCameraShared(size_t camera_index) const;

            /// How many cameras does this system have?
            inline size_t numCameras() const { return cameras_.size(); }

            /// Print camera infos
            void printParameters(std::ostream &out, const std::string &s = "Camera: ") const;

            /// Get all cameras.
            const std::vector<std::shared_ptr<Camera>> &getCameraVector() const;

            void maskProcess(const int idx, cv::Mat &img);

            void photometricUndistorter(const int idx, cv::Mat &img);

            /// Get a label for the camera.
            inline const std::string &getLabel() const { return label_; }

            /// Set a label for the camera.
            inline void setLabel(const std::string &label) { label_ = label; }

            /// keep first N cameras
            inline void keepFirstNCams(const int N)
            {
                CHECK_LT(N, static_cast<int>(cameras_.size()));
                CHECK_GT(N, 0);
                T_C_B_.erase(T_C_B_.begin() + N, T_C_B_.end());
                cameras_.erase(cameras_.begin() + N, cameras_.end());
            }

        private:
            /// Internal consistency checks and initialization.
            void initInternal();

            /// The mounting transformations.
            TransformationVector T_C_B_;

            /// The camera geometries.
            std::vector<std::shared_ptr<Camera>> cameras_;

            /// A label for this camera rig, a name.
            std::string label_;
        };

    } // namespace cameras
} // namespace vikit
