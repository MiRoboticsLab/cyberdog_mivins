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

#include <glog/logging.h>
#include <mivins/common/types.h>

namespace mivins
{

    // -----------------------------------------------------------------------------
    struct FeatureTrackerOptions
    {
        /// We do the Lucas Kanade tracking in a pyramidal way. max_level specifies the
        /// coarsest pyramidal level to optimize. For an image resolution of (640x480)
        /// we set this variable to 4 if you have an image with double the resolution,
        /// increase this number by one.
        int klt_max_level = 4;

        /// Similar to klt_max_level, this is the coarsest level to search for.
        /// if you have a really high resolution image and you don't extract
        /// features down to the lowest level you can set this number larger than 0.
        int klt_min_level = 0;

        /// Patch-size to use on each pyramid level.
        std::vector<int> klt_patch_sizes = {16, 16, 16, 8, 8};

        /// KLT termination criterion.
        int klt_max_iter = 30;

        /// KLT termination criterion.
        double klt_min_update_squared = 0.001;

        /// Use the first observation as klt template. If set to false, then the
        /// last observation is used, which results in more feature drift.
        bool klt_template_is_first_observation = true;

        /// If number of tracks falls below this threshold, detect new features.
        size_t min_tracks_to_detect_new_features = 50;

        /// Reset tracker before detecting new features. This means that all active
        /// tracks are always the same age.
        bool reset_before_detection = true;

        int init_features_size = 150;
        float init_disparity = 20.0;
        double init_disparity_pivot_ratio = 0.5;
    };

    // -----------------------------------------------------------------------------
    class FeatureObs
    {
    public:
        FeatureObs() = delete;

        FeatureObs(const FrameBundlePtr &frame_bundle, size_t camera_index, size_t feature_index);

        inline const FrameBundlePtr GetFrameBundle() const
        {
            return m_frame_bundle;
        }

        inline size_t GetFrameIndex() const
        {
            return m_camera_index;
        }

        inline size_t GetFeatureIndex() const
        {
            return m_feature_index;
        }

        const Eigen::Block<Keypoints, 2, 1> GetPx() const;

        // const Eigen::Block<Bearings, 3, 1> GetBearing() const;

        const FramePtr GetFrame() const;

    private:
        FrameBundlePtr m_frame_bundle;
        size_t m_camera_index;
        size_t m_feature_index;
    };

    typedef std::vector<FeatureObs> FeatureObsPerIdList;

    class FeatureTrackPerId
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        explicit FeatureTrackPerId(int track_id);

        inline int GetTrackId() const
        {
            return m_track_id;
        }

        inline const FeatureObsPerIdList &GetFeatureTrack() const
        {
            return m_feature_track_list;
        }

        /*inline size_t Size() const 
  {
    return m_feature_track_list.size();
  }*/

        inline bool empty() const
        {
            return m_feature_track_list.empty();
        }

        /// The feature at the front is the first observed feature.
        inline const FeatureObs &front() const
        {
            CHECK(!empty()) << "Track empty when calling front().";
            return m_feature_track_list.front();
        }

        /// The feature at the back is the last observed feature.
        inline const FeatureObs &back() const
        {
            CHECK(!empty()) << "Track empty when calling back().";
            return m_feature_track_list.back();
        }

        inline const FeatureObs &at(size_t i) const
        {
            CHECK_LT(i, m_feature_track_list.size()) << "Index too large.";
            return m_feature_track_list.at(i);
        }

        /// New observations are always inserted at the back of the vector.
        inline void pushBack(
            const FrameBundlePtr &frame_bundle,
            const size_t camera_index,
            const size_t feature_index)
        {
            m_feature_track_list.emplace_back(FeatureObs(frame_bundle, camera_index, feature_index));
        }

        double GetDisparity() const;

    private:
        int m_track_id;
        FeatureObsPerIdList m_feature_track_list;
    };

    typedef std::vector<FeatureTrackPerId,
                        Eigen::aligned_allocator<FeatureTrackPerId>>
        FeatureTracksPerCam;

} // namespace mivins
