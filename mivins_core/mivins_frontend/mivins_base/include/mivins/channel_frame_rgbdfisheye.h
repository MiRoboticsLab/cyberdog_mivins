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
#include <mivins/tracker/feature_tracker.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

namespace mivins
{

    class ChannelFrameRgbdFisheye : public ChannelFrameBase
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef std::shared_ptr<ChannelFrameRgbdFisheye> Ptr;

        /// Default constructor
        ChannelFrameRgbdFisheye(
            const BaseOptions &base_options,
            const DepthOptimizationOptions &depth_filter_options,
            const DetectorOptions &feature_detector_options,
            const InitializationOptions &init_options,
            const ReprojectorOptions &reprojector_options,
            const FeatureTrackerOptions &tracker_options,
            const CameraBundle::Ptr &cam);

        virtual ~ChannelFrameRgbdFisheye() = default;

        void AddImage(
            const cv::Mat &img,
            const uint64_t timestamp);

        /// @}

        inline CameraPtr cam() const { return cams_->getCameraShared(0); }

    protected:
        /// Pipeline implementation. Called by base class.
        virtual UpdateResult ProcessFrameBundle() override;

        virtual UpdateResult JointInitialize();

        /// Processes the first frame and sets it as a keyframe.
        //virtual UpdateResult ProcessFirstFrame();

        /// Processes all frames after the first two keyframes.
        virtual UpdateResult ProcessFrame();
        virtual UpdateResult MakeKeyframe(const size_t camera_id);
        virtual UpdateResult ProcessFrameMonoVersion();
        ///use vins initialziation result
        virtual UpdateResult ProcessVinsFrame();

        /// Try relocalizing the frame at relative position to provided keyframe.
        virtual UpdateResult RelocalizeFrame(
            const Transformation &T_cur_ref,
            const FramePtr &ref_keyframe);

        virtual UpdateResult RelocalizeFramebundle(
            const Transformation & /*T_cur_ref*/,
            const FrameBundlePtr &ref_keyframe);

        /// Reset the frame handler. Implement in derived class.
        virtual void ResetAll() override;

        virtual int GetType();

    private:
        bool init_ = false;
        int depth_cam_id_ = 0;
        int init_landmark_size_ = 10;
        float init_disparity_ = 20.0f;

        double reproj_error_thresh_ = 2.0f;
        FeatureTracker::Ptr tracker_;
        DetectorPtr feature_detector_;
        FeatureTrackerOptions tracker_options_;

        bool AddInitFrameBundle(const mivins::FrameBundlePtr &new_frames);

        void GetMotionPriorWithDepth();
        int PnPRansac(
            const std::vector<cv::Point2f> &matched_2d,
            const std::vector<cv::Point3f> &matched_3d,
            Transformation &T_2d_3d);

        /*void drawFeatureMatches(
    const FramePtr &frame1, const FramePtr &frame2,
    std::unordered_map<size_t, size_t> &matches_12);
  bool trackFeaturesAndCheckDisparity(const FrameBundlePtr& frames);
  void getFeatureMatches(
    const FramePtr &frame1, const FramePtr &frame2,
    std::unordered_map<size_t, size_t> &matches_12);*/
        bool GetRelPoseWithPnPRansac(
            const FramePtr &frame_cur, const FramePtr &frame_ref,
            Transformation &T_cur_ref);
        //void trackNewFeatures(const FrameBundlePtr& frames);
        bool TriangulateAndInitializePoints(
            const FrameBundlePtr &frames_cur, const FrameBundlePtr &frames_ref);
        void RejectWithF(
            const FramePtr &frame1, const FramePtr &frame2,
            std::unordered_map<size_t, size_t> &matches_12);
        void TriangulatePoints(
            const FramePtr &frame_cur, const FramePtr &frame_ref,
            std::unordered_map<size_t, size_t> &matches_cur_ref,
            std::unordered_map<size_t, cv::Point3f> &points_in_world);
        void InitializePoints(
            const FramePtr &frame_cur, const FramePtr &frame_ref,
            const std::unordered_map<size_t, size_t> &matches_cur_ref,
            const std::unordered_map<size_t, cv::Point3f> &points_in_world);

        //bool trackFeatures_rgbd2fish(const FrameBundlePtr& frames);
        UpdateResult InitFrontend();
    };

} // namespace mivins
