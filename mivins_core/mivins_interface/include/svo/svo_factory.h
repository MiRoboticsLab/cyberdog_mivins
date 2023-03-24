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
#include <mivins/common/camera_fwd.h>

namespace mivins
{

    // forward declarations
    class ChannelImu;
    class ChannelOdom;
    class LooseCouple;
    class GlobalMap;
    class FrameHandlerRgbd;
    class ChannelFrameMono;
    class ChannelFrameStereo;
    class FrameHandlerTripleWithStereo;
    class FrameHandlerDenseMono;
    class FrameHandlerTripleWithDepth;
    class ChannelFrameRgbdFisheye;
    class PoseUpdate;

    namespace factory
    {

        /// Get IMU Handler.
        std::shared_ptr<ChannelImu> getImuHandler(
            const std::string config_file,
            const std::string calib_file);

        /// Get ODOM Handler.
        std::shared_ptr<ChannelOdom> getOdomHandler(
            const std::string calib_file);
        
        /// Get LooseCouple Handler    
        std::shared_ptr<LooseCouple> getLooseCoupleHandler(
             const std::string config_file, 
             const std::string calib_file);

        /// Get PoseUpdate Handler
        std::shared_ptr<PoseUpdate> getPoseUpdateHandler(
            const std::string config_file);

        /// Factory for Rgbd-SVO.
        // std::shared_ptr<FrameHandlerRgbd> makeRgbd(
        //     const std::string config_file,
        //     const std::string calib_file,
        //     const CameraBundlePtr& cam = nullptr);

        /// Factory for Mono-SVO.
        std::shared_ptr<ChannelFrameMono> makeMono(
            const std::string config_file,
            const std::string calib_file,
            const CameraBundlePtr& cam = nullptr);

        /// Factory for Stereo-SVO.
        std::shared_ptr<ChannelFrameStereo> makeStereo(
            const std::string config_file,
            const std::string calib_file,
            const CameraBundlePtr& cam = nullptr);

        /// Factory for Triple with Stereo-SVO
        std::shared_ptr<FrameHandlerTripleWithStereo> makeTripleWithStereo(
            const std::string config_file,
            const std::string calib_file,
            const CameraBundlePtr& cam = nullptr);

        /// Factory for Triple with depth-SVO
        std::shared_ptr<ChannelFrameRgbdFisheye> makeTripleWithDepth(
            const std::string config_file,
            const std::string calib_file,
            const CameraBundlePtr& cam = nullptr);

    } // namespace factory
} // namespace mono
