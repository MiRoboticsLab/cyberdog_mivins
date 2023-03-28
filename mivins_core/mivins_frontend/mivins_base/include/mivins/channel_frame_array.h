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

    class ChannelFrameArray : public ChannelFrameBase
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef std::shared_ptr<ChannelFrameArray> Ptr;

        /// Default constructor
        ChannelFrameArray(
            const BaseOptions &base_options,
            const DepthOptimizationOptions &depth_filter_options,
            const DetectorOptions &feature_detector_options,
            const InitializationOptions &init_options,
            const ReprojectorOptions &reprojector_options,
            const FeatureTrackerOptions &tracker_options,
            const CameraBundle::Ptr &cameras);

        virtual ~ChannelFrameArray() = default;

        // deprecated. use AddImageBundle().
        void AddImages(
            const std::vector<cv::Mat> &images,
            const uint64_t timestamp);

        const FrameBundlePtr &LastFrames() const
        {
            return last_frames_;
        }

    protected:
        /// Pipeline implementation. Called by base class.
        virtual UpdateResult ProcessFrameBundle() override;

        UpdateResult ProcessFirstFrame();

        UpdateResult ProcessSecondFrame();

        UpdateResult ProcessFrame();

        UpdateResult MakeKeyframe(const size_t camera_id);

        /// Reset the frame handler. Implement in derived class.
        virtual void ResetAll() override;
        virtual int GetType();
    };

} // namespace mivins
