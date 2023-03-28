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

#include <mivins/common/frame.h>
#include <mivins/tracker/feature_tracker_obs.h>

namespace mivins
{

    // -----------------------------------------------------------------------------
    FeatureObs::FeatureObs(
        const FrameBundlePtr &frame_bundle, size_t camera_index, size_t feature_index)
        : m_frame_bundle(frame_bundle), m_camera_index(camera_index), m_feature_index(feature_index)
    {
        CHECK_LT(m_camera_index, m_frame_bundle->size());
    }

    const Eigen::Block<Keypoints, 2, 1> FeatureObs::GetPx() const
    {
        return m_frame_bundle->at(m_camera_index)->px_vec_.block<2, 1>(0, m_feature_index);
    }

    // const Eigen::Block<Bearings, 3, 1> FeatureObs::GetBearing() const
    // {
    //   return m_frame_bundle->at(m_camera_index)->f_vec_.block<3,1>(0, m_feature_index);
    // }

    const FramePtr FeatureObs::GetFrame() const
    {
        return m_frame_bundle->at(m_camera_index);
    }

    // -----------------------------------------------------------------------------
    FeatureTrackPerId::FeatureTrackPerId(int track_id)
        : m_track_id(track_id)
    {
        m_feature_track_list.reserve(10);
    }

    double FeatureTrackPerId::GetDisparity() const
    {
        return (front().GetPx() - back().GetPx()).norm();
    }

} // namespace mivins
