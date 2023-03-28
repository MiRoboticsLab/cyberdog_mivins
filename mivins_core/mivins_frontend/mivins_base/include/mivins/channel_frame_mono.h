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

#include <mivins/channel_frame_base.h>

namespace mivins
{

    /// Monocular Semi-Direct Visual Odometry Pipeline
    ///
    /// References:
    /// [1] Christian Forster, Matia Pizzoli, Davide Scaramuzza, "SVO: Semi-Direct
    /// Monocular Visual Odometry for Micro Aerial Vehicles", Proc. IEEE International
    /// Conference on Robotics and Automation, Honkong 2015.
    ///
    /// This is the main interface class of the VO. It derives from ChannelFrameBase
    /// which maintains the state machine (start, stop, reset).
    class ChannelFrameMono : public ChannelFrameBase
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef std::shared_ptr<ChannelFrameMono> Ptr;

        /// Default constructor
        ChannelFrameMono(
            const BaseOptions &base_options,
            const DepthOptimizationOptions &depth_filter_options,
            const DetectorOptions &feature_detector_options,
            const InitializationOptions &init_options,
            const ReprojectorOptions &reprojector_options,
            const FeatureTrackerOptions &tracker_options,
            const CameraBundle::Ptr &cam);

        virtual ~ChannelFrameMono() = default;

        /// @name Main Interface
        ///
        /// These are the main functions that you will need to interface
        /// with the visual odometry pipeline. Other important functions like
        /// start(), reset(), stage() are provided in the ChannelFrameBase class which
        /// maintains the state-machine.
        ///
        /// @{

        /// Provide an image to the odometry pipeline
        /// \param img A 8bit grayscale OpenCV image.
        /// \param timestamp Corresponding timestamp of the image. Is saved in the
        /// frame class.
        /// \param R_origin_imu If you use a IMU gyroscope and you integrate it outside
        /// of SVO, you can provide the orientation here. In order to be used, you have
        /// to set option->use_provided_attitude to true. If you set the priors in the
        /// options larger than one the gyroscope measurement is used as weighted prior
        /// in the optimizations. Note that you have to provide the orientation of the
        /// IMU frame and not the camera. The relative transformation between imu and
        /// camera is set inside cam_. Another way to provide an orientation estimate
        /// is to use set the imuHandler and to provide it with rotation rate
        /// measurements.
        /// \param custom_id If you would like to save an identifier key with the the
        /// frame you can provide it here. It is not used by SVO for anything but can
        /// be accessed with frame->customId().
        //! @deprecated. use AddImageBundle().
        void AddImage(
            const cv::Mat &img,
            const uint64_t timestamp);

        /// After adding an image to SVO with AddImage(), the image is saved in a
        /// Frame class, the frame is processed and it's pose computed. Most likely you
        /// want to know the pose of the cam. Therefore, use this function to access
        /// the pose of the last provided image (e.g., vo->LastFrame()->T_world_cam()).
        /// \return FramePtr !!!IMPORTANT!!! can be nullptr if something did not work
        /// well or if you call this function when pipeline is not running, i.e.,
        /// vo->stage()==STAGE_PAUSED.
        FramePtr LastFrame() const;

        /// @}

        inline CameraPtr cam() const { return cams_->getCameraShared(0); }

    protected:
        // helpers
        const FramePtr &NewFrame() const;

        // unsafe because last_frame might be nullptr. use this function only
        // when you are sure that it is set!
        const FramePtr &LastFrameUnsafe() const;

        bool HaveLastFrame() const;

        /// Pipeline implementation. Called by base class.
        virtual UpdateResult ProcessFrameBundle() override;

        virtual UpdateResult JointInitialize();

        /// Processes the first frame and sets it as a keyframe.
        virtual UpdateResult ProcessFirstFrame();

        ///use vins initialziation result
        virtual UpdateResult ProcessVinsFrameMono();

        /// Processes all frames after the first two keyframes.
        virtual UpdateResult ProcessFrame();

        /// Try relocalizing the frame at relative position to provided keyframe.
        virtual UpdateResult RelocalizeFrame(
            const Transformation &T_cur_ref,
            const FramePtr &ref_keyframe);

        /// Reset the frame handler. Implement in derived class.
        virtual void ResetAll() override;
        virtual int GetType();
        UpdateResult TryToInitialize();
        UpdateResult MakeKeyFrame();
    };

} // namespace mivins
