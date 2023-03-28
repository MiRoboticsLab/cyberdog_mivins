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

#include <string>
#include <memory>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <mivins/utils/transformation_utils.h>

namespace vk
{

    using Transformation = mivins::Transformation;
    using TransformationVector = mivins::TransformationVector;
    using Quaternion = mivins::Quaternion;

    namespace cameras
    {

        /// \struct ProjectionResult
        /// \brief This struct is returned by the camera projection methods and holds the result state
        ///        of the projection operation.
        struct ProjectionResult
        {
            /// Possible projection state.
            enum class Status
            {
                /// Keypoint is visible and projection was successful.
                KEYPOINT_VISIBLE,
                /// Keypoint is NOT visible but projection was successful.
                KEYPOINT_OUTSIDE_IMAGE_BOX,
                /// The projected point lies behind the camera plane.
                POINT_BEHIND_CAMERA,
                /// The projection was unsuccessful.
                PROJECTION_INVALID,
                /// Default value after construction.
                UNINITIALIZED
            };
            // Make the enum values accessible from the outside without the additional indirection.
            static Status KEYPOINT_VISIBLE;
            static Status KEYPOINT_OUTSIDE_IMAGE_BOX;
            static Status POINT_BEHIND_CAMERA;
            static Status PROJECTION_INVALID;
            static Status UNINITIALIZED;

            constexpr ProjectionResult() : status_(Status::UNINITIALIZED) {}
            constexpr ProjectionResult(Status status) : status_(status) {}

            /// \brief ProjectionResult can be typecasted to bool and is true if the projected keypoint
            ///        is visible. Simplifies the check for a successful projection.
            ///        Example usage:
            /// @code
            ///          aslam::ProjectionResult ret = camera_->project3(Eigen::Vector3d(0, 0, -10), &keypoint);
            ///          if(ret) std::cout << "Projection was successful!\n";
            /// @endcode
            explicit operator bool() const { return isKeypointVisible(); }

            /// \brief Compare objects.
            bool operator==(const ProjectionResult &other) const { return status_ == other.status_; }

            /// \brief Compare projection status.
            bool operator==(const ProjectionResult::Status &other) const { return status_ == other; }

            /// \brief Convenience function to print the state using streams.
            friend std::ostream &operator<<(std::ostream &out, const ProjectionResult &state);

            /// \brief Check whether the projection was successful and the point is visible in the image.
            bool isKeypointVisible() const { return (status_ == Status::KEYPOINT_VISIBLE); }

            /// \brief Returns the exact state of the projection operation.
            ///        Example usage:
            /// @code
            ///          aslam::ProjectionResult ret = camera_->project3(Eigen::Vector3d(0, 0, -1), &keypoint);
            ///          if(ret.getDetailedStatus() == aslam::ProjectionResult::Status::KEYPOINT_OUTSIDE_IMAGE_BOX)
            ///            std::cout << "Point behind camera! Lets do something...\n";
            /// @endcode
            Status getDetailedStatus() const { return status_; }

        private:
            /// Stores the projection state.
            Status status_;
        };

        class CameraGeometryBase
        {
        public:
            typedef std::shared_ptr<CameraGeometryBase> Ptr;
            typedef std::shared_ptr<const CameraGeometryBase> ConstPtr;

            enum class Type
            {
                kPinhole = 0,
                kUnifiedProjection = 1,
                kOmni = 2,
                kEqFisheye = 3,
                kMei = 4
            };

            /// Default constructor
            CameraGeometryBase(const int width, const int height);

            virtual ~CameraGeometryBase() = default;

            /// Load a camera rig form a yaml file. Returns a nullptr if the loading fails.
            static Ptr loadFromYaml(const std::string &yaml_file);

            /// Computes bearing vector from pixel coordinates. Z-component of the returned
            /// bearing vector is 1.0. IMPORTANT: returned vector is NOT of unit length!
            virtual bool backProject3(
                const Eigen::Ref<const Eigen::Vector2d> &keypoint,
                Eigen::Vector3d *out_point_3d) const = 0;
            /// Override taking multiple keypoints. Derived classes are free to make a
            /// vectorized implementation for speedup.
            virtual void backProject3(
                const Eigen::Ref<const Eigen::Matrix2Xd> &keypoints,
                Eigen::Matrix3Xd *out_bearing_vectors, std::vector<bool> *success) const;

            /// Computes pixel coordinates from bearing vector with Jacobian w.r.t. point.
            virtual const ProjectionResult project3(
                const Eigen::Ref<const Eigen::Vector3d> &point_3d,
                Eigen::Vector2d *out_keypoint,
                Eigen::Matrix<double, 2, 3> *out_jacobian_point = nullptr) const = 0;

            /// Print camera info
            virtual void printParameters(std::ostream &out, const std::string &s = "Camera: ") const = 0;

            /// Equivalent to focal length for projective cameras and factor for
            /// omni-directional cameras that transforms small angular error to pixel-error.
            virtual double errorMultiplier() const = 0;

            /// Get Intrinsic parameters from camera
            virtual Eigen::VectorXd getIntrinsicParameters() const = 0;

            /// Get Distortion parameters from camera
            virtual Eigen::VectorXd getDistortionParameters() const = 0;

            virtual double GetAngleError(double img_err) const = 0;

            /// CameraType value representing the camera model used by the derived class.
            inline Type GetType() const { return camera_type_; }

            /// Name of the camera.
            inline const std::string &getLabel() const { return label_; }

            /// Set user-specific camera label.
            inline void setLabel(const std::string &label) { label_ = label; }

            /// Image width in pixels.
            uint32_t imageWidth() const { return width_; }

            /// Image height in pixels.
            uint32_t imageHeight() const { return height_; }

            /// Return if a given keypoint is inside the imaging box of the camera.
            template <typename DerivedKeyPoint>
            bool isKeypointVisible(const Eigen::MatrixBase<DerivedKeyPoint> &keypoint) const;

            /// Return if a given keypoint is within the specified margin to the boundary
            /// of the imaging box of the camera.
            template <typename DerivedKeyPoint>
            bool isKeypointVisibleWithMargin(
                const Eigen::MatrixBase<DerivedKeyPoint> &keypoint,
                typename DerivedKeyPoint::Scalar margin) const;

            /// Set the mask. Masks must be the same size as the image and they follow the same
            /// convention as OpenCV: 0 == masked, nonzero == valid.
            void setMask(const cv::Mat &mask);

            /// Get the mask.
            inline const cv::Mat &GetMask() const { return mask_; }

            /// Clear the mask.
            inline void clearMask() { mask_ = cv::Mat(); }

            /// Does the camera have a mask?
            inline bool hasMask() const { return !mask_.empty(); }

            /// load from file
            void loadMask(const std::string &mask_file);

            /// Check if the keypoint is masked.
            bool isMasked(const Eigen::Ref<const Eigen::Vector2d> &keypoint) const;

            // process input image with mask
            void maskProcess(cv::Mat &img)
            {
            }

            // photometric undistorter for the input image
            void photometricUndistorter(cv::Mat &img);

            /// Creates a random non-masked keypoint.
            Eigen::Vector2d createRandomKeypoint() const;

            void setDepthMin(const float depth_min);

            void setDepthMax(const float depth_max);

            void setDepthScale(const float depth_scale);

            float getDepthMin();

            float getDepthMax();

            float getDepthScale();

            void loadVignette(const std::string &vignette_file);

            void loadGamma(const std::string &gamma_file);

            bool bgamma_;
            bool bvignette_;

        protected:
            int width_;
            int height_;
            std::string label_;
            Type camera_type_;
            cv::Mat mask_;
            float depth_min_;
            float depth_max_;
            float depth_scale_;

            float *vignette_map_inv_;
            float gamma_data_[256 * 256];
        };

    } // namespace cameras
} // namespace vk

#include "mivins/camera_models/camera_geometry_base.hpp"
