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

#include <unordered_map>
#include <deque>
#include <mutex>
#include <mivins/mivins_global_types.h>

namespace mivins
{

    /// FrontendLocalMap object which saves all keyframes which are in a map.
    class FrontendLocalMap
    {
    public:
        typedef std::shared_ptr<FrontendLocalMap> Ptr;
        typedef std::unordered_map<int, FramePtr> Keyframes; // Frame-Id & Pointer

        Keyframes keyframes_; //!< List of keyframes in the map.
        std::vector<PointPtr> points_to_delete_;
        std::mutex points_to_delete_mutex_;
        int last_added_kf_id_;
        std::deque<int> sorted_keyframe_ids_; //!< Used with ceres backend

        // cache the last removed keyframe for use of other modules
        FramePtr last_removed_kf_ = nullptr;

        FrontendLocalMap();
        ~FrontendLocalMap();

        FrontendLocalMap(const FrontendLocalMap &) = delete;            // no copy
        FrontendLocalMap &operator=(const FrontendLocalMap &) = delete; // no copy

        /// Reset the map. Delete all keyframes and reset the frame and point counters.
        void reset();

        /// Add a new keyframe to the map.
        void AddKeyframe(const FramePtr &new_keyframe, bool temporal_map);

        /// Moves the frame to the trash queue which is cleaned now and then.
        void RemoveKeyframe(const int frame_id);

        /// Remove oldest keyframe (used with temporal map)
        void RemoveOldestKeyframe();

        /// Safely remove a point from a map
        void SafeDeletePoint(const PointPtr &pt);

        /// Moves the point to the trash bin, cleared after all reprojectors have done their FilterJob
        void AddPointToTrash(const PointPtr &pt);

        /// Effectively delete points in the trash bin
        void EmptyPointsTrash();

        /// Given a frame, return all keyframes which have an overlapping field of view.
        void GetOverlapKeyframes(
            const FramePtr &frame,
            std::vector<std::pair<FramePtr, double>> *close_kfs) const;

        /// Given a frame, return N closest keyframes with overlapping FoV.
        void GetClosestNKeyframesWithOverlap(
            const FramePtr &cur_frame,
            const size_t num_frames,
            std::vector<FramePtr> *visible_kfs) const;

        /// Return the keyframe which is spatially closest and has overlapping field of view.
        FramePtr GetClosestKeyframe(const FramePtr &frame) const;

        /// Return the keyframe which is oldest
        FramePtr GetOldsestKeyframe() const;

        /// Return the keyframe which is furthest apart from pos.
        FramePtr GetFurthestKeyframe(const Vector3d &pos) const;

        /// Get Keyframe by Frame-Id. Used for relocalizer.
        FramePtr GetKeyframeById(const int id) const;

        /// We use a hashtable to store the keyframes. This function provides a sorted
        /// list of keyframes.
        void GetSortedKeyframes(std::vector<FramePtr> &kfs_sorted) const;

        /// Transform the whole map with rotation R, translation t and scale s.
        void transform(const Matrix3d &R, const Vector3d &t, const double &s);

        /// Return the number of keyframes in the map
        inline size_t size() const { return keyframes_.size(); }

        /// Returns the last added keyframe (or a nullptr if none has been added)
        inline FramePtr GetLastKeyframe() { return this->GetKeyframeById(last_added_kf_id_); }

        void CheckDataConsistency() const;

        inline FramePtr GetKeyFrameAt(const size_t i)
        {
            return this->GetKeyframeById(sorted_keyframe_ids_[i]);
        }

        inline size_t NumKeyframes() const
        {
            return keyframes_.size();
        }

        inline const FramePtr GetKeyFrameAt(const size_t i) const
        {
            return this->GetKeyframeById(sorted_keyframe_ids_[i]);
        }

        void GetLastRemovedKF(FramePtr *f);
    };

} // namespace mivins
