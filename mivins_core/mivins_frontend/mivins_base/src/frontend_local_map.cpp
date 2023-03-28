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

#include <set>
#include <algorithm>
#include <mivins/frontend_local_map.h>
#include <mivins/common/point.h>
#include <mivins/common/frame.h>

namespace mivins
{

    FrontendLocalMap::FrontendLocalMap()
        : last_added_kf_id_(-1)
    {
    }

    FrontendLocalMap::~FrontendLocalMap()
    {
        reset();
        LOG_INFO_STREAM("FrontendLocalMap destructed");
    }

    void FrontendLocalMap::reset()
    {
        keyframes_.clear();
        sorted_keyframe_ids_.clear();
        last_added_kf_id_ = -1;
        points_to_delete_.clear();
    }

    void FrontendLocalMap::RemoveKeyframe(const int frame_id)
    {
        auto it_kf = keyframes_.find(frame_id);
        if (it_kf == keyframes_.end())
        {
            LOG(WARNING) << "Cannot find the keyframe with id " << frame_id
                         << ", will not do anything.";
            return;
        }
        const FramePtr &frame = it_kf->second;
        last_removed_kf_ = frame;
        for (size_t i = 0; i < frame->num_features_; ++i)
        {
            if (frame->landmark_vec_[i])
            {
                // observation is used in frontend to find reference frame, etc
                // this frame is not going to be used in the front end
                frame->landmark_vec_[i]->RemoveObservation(frame_id);
                // Since the map is the first to remove keyframe,
                //  the frame may still be used by other module
                // frame->landmark_vec_[i] = nullptr;
            }
        }
        keyframes_.erase(it_kf);
    }

    void FrontendLocalMap::RemoveOldestKeyframe()
    {
        RemoveKeyframe(sorted_keyframe_ids_.front());
        sorted_keyframe_ids_.pop_front();
    }

    void FrontendLocalMap::SafeDeletePoint(const PointPtr &pt)
    {
        // Delete references to mappoints in all keyframes
        for (const Point3dObservation &obs : pt->obs_)
        {
            if (const FramePtr &frame = obs.frame.lock())
            {
                frame->DeleteLandmark(obs.keypoint_index);
            }
            else
                LOG_ERROR_STREAM("could not lock weak_ptr<Frame> in FrontendLocalMap::SafeDeletePoint");
        }
        pt->obs_.clear();
    }

    void FrontendLocalMap::AddPointToTrash(const PointPtr &pt)
    {
        std::lock_guard<std::mutex> lock(points_to_delete_mutex_);
        points_to_delete_.push_back(pt);
    }

    void FrontendLocalMap::EmptyPointsTrash()
    {
        std::lock_guard<std::mutex> lock(points_to_delete_mutex_);
        LOG_DEBUG_STREAM("Deleting " << points_to_delete_.size() << " point from trash");
        for (auto &pt : points_to_delete_)
        {
            SafeDeletePoint(pt);
        }
        points_to_delete_.clear();
    }

    void FrontendLocalMap::AddKeyframe(const FramePtr &new_keyframe, bool temporal_map)
    {
        VLOG(100) << "Adding keyframe to map. Frame-Id = " << new_keyframe->GetFrameId();

        keyframes_.insert(std::make_pair(new_keyframe->GetFrameId(), new_keyframe));
        last_added_kf_id_ = new_keyframe->GetFrameId();
        if (true) //temporal_map)
        {
            sorted_keyframe_ids_.push_back(new_keyframe->GetFrameId());
        }
    }

    void FrontendLocalMap::GetOverlapKeyframes(
        const FramePtr &frame,
        std::vector<std::pair<FramePtr, double>> *close_kfs) const
    {
        CHECK_NOTNULL(close_kfs);
        for (const auto &kf : keyframes_)
        {
            // check first if Point is visible in the Keyframe, use therefore KeyPoints
            for (const auto &keypoint : kf.second->key_pts_)
            {
                if (keypoint.first == -1)
                    continue;

                if (frame->IsVisible(keypoint.second))
                {
                    close_kfs->push_back(
                        std::make_pair(kf.second,
                                       (frame->T_f_w_.GetPosition() - kf.second->T_f_w_.GetPosition()).norm()));
                    break; // this keyframe has an overlapping field of view -> add to close_kfs
                }
            }
        }
    }

    void FrontendLocalMap::GetClosestNKeyframesWithOverlap(
        const FramePtr &cur_frame,
        const size_t num_frames,
        std::vector<FramePtr> *close_kfs) const
    {
        CHECK_NOTNULL(close_kfs);
        std::vector<std::pair<FramePtr, double>> overlap_kfs;
        GetOverlapKeyframes(cur_frame, &overlap_kfs);
        if (overlap_kfs.empty())
            return;

        size_t N = std::min(num_frames, overlap_kfs.size());
        // Extract closest N frames.
        std::nth_element(
            overlap_kfs.begin(), overlap_kfs.begin() + N, overlap_kfs.end(),
            [](const std::pair<FramePtr, double> &lhs,
               const std::pair<FramePtr, double> &rhs)
            { return lhs.second < rhs.second; });
        overlap_kfs.resize(N);

        close_kfs->reserve(num_frames);
        std::transform(overlap_kfs.begin(), overlap_kfs.end(), std::back_inserter(*close_kfs),
                       [](const std::pair<FramePtr, double> &p)
                       { return p.first; });
    }

    FramePtr FrontendLocalMap::GetClosestKeyframe(const FramePtr &frame) const
    {
        CHECK_NOTNULL(frame.get());
        std::vector<std::pair<FramePtr, double>> close_kfs;
        GetOverlapKeyframes(frame, &close_kfs);
        if (close_kfs.empty())
            return nullptr;
        std::sort(
            close_kfs.begin(), close_kfs.end(),
            [](const std::pair<FramePtr, double> &lhs,
               const std::pair<FramePtr, double> &rhs)
            { return lhs.second < rhs.second; });
        if (close_kfs.at(0).first != frame)
            return close_kfs.at(0).first;
        if (close_kfs.size() == 1)
            return nullptr;
        return close_kfs.at(1).first;
    }

    FramePtr FrontendLocalMap::GetOldsestKeyframe() const
    {
        return GetKeyFrameAt(0);
    }

    FramePtr FrontendLocalMap::GetFurthestKeyframe(const Vector3d &pos) const
    {
        FramePtr furthest_kf;
        double maxdist = 0.0;
        for (const auto &kf : keyframes_)
        {
            double dist = (kf.second->GetCameraPosInWorld() - pos).norm();
            if (dist > maxdist)
            {
                maxdist = dist;
                furthest_kf = kf.second;
            }
        }
        return furthest_kf;
    }

    FramePtr FrontendLocalMap::GetKeyframeById(const int id) const
    {
        auto it_kf = keyframes_.find(id);
        if (it_kf == keyframes_.end())
            return nullptr;
        return it_kf->second;
    }

    void FrontendLocalMap::GetSortedKeyframes(std::vector<FramePtr> &kfs_sorted) const
    {
        kfs_sorted.reserve(keyframes_.size());
        std::transform(
            keyframes_.begin(), keyframes_.end(), std::back_inserter(kfs_sorted),
            [](const std::pair<int, FramePtr> &val)
            { return val.second; });
        std::sort(
            kfs_sorted.begin(), kfs_sorted.end(),
            [](const FramePtr &left, const FramePtr &right)
            { return left->GetFrameId() < right->GetFrameId(); });
    }

    void FrontendLocalMap::transform(const Matrix3d &R, const Vector3d &t, const double &s)
    {
        for (const auto &kf : keyframes_)
        {
            Vector3d pos = s * R * kf.second->GetCameraPosInWorld() + t;
            Matrix3d rot = R * kf.second->T_f_w_.GetRotation().Inverse().GetRotationMatrix();
            kf.second->T_f_w_ = Transformation(Quaternion(rot), pos).Inverse();
            CHECK(false);
            /* TODO(cfo)
    for(const auto& ftr : kf.second->fts_)
    {
      if(ftr->point->last_published_ts_ == 0)
        continue;
      ftr->point->last_published_ts_ = 0;
      ftr->point->pos3d_in_w = s*R*ftr->point->pos3d_in_w + t;
    }
    */
        }
    }

    void FrontendLocalMap::CheckDataConsistency() const
    {
        for (const auto &kf : keyframes_)
        {
            const FramePtr &frame = kf.second;

            // check that feature-stuff has all same length
            CHECK_EQ(frame->px_vec_.cols(), frame->f_vec_.cols());
            CHECK_EQ(frame->px_vec_.cols(), frame->level_vec_.size());
            CHECK_EQ(frame->px_vec_.cols(), frame->grad_vec_.cols());
            CHECK_EQ(static_cast<size_t>(frame->px_vec_.cols()), frame->type_vec_.size());
            CHECK_EQ(static_cast<size_t>(frame->px_vec_.cols()), frame->landmark_vec_.size());
            CHECK_EQ(static_cast<size_t>(frame->px_vec_.cols()), frame->seed_ref_vec_.size());
            CHECK_EQ(frame->px_vec_.cols(), frame->invmu_sigma2_a_b_vec_.cols());
            CHECK_LE(frame->num_features_, static_cast<size_t>(frame->px_vec_.cols()));

            // check features
            for (size_t i = 0; i < frame->num_features_; ++i)
            {
                if (frame->landmark_vec_[i])
                {
                    // make sure that the 3d point has only one reference back
                    size_t ref_count = 0;
                    for (const Point3dObservation &o : frame->landmark_vec_[i]->obs_)
                    {
                        if (o.frame_id == frame->GetFrameId())
                        {
                            ++ref_count;
                            const FramePtr obs_frame = o.frame.lock();
                            CHECK_EQ(obs_frame.get(), frame.get());
                        }
                    }
                    CHECK_EQ(ref_count, 1u);
                }

                CHECK((isSeed(frame->type_vec_[i]) && frame->landmark_vec_[i] == nullptr) || !isSeed(frame->type_vec_[i]));
                CHECK(frame->seed_ref_vec_[i].keyframe == nullptr);
                CHECK(frame->seed_ref_vec_[i].seed_id == -1);
            }
        }
    }

    void FrontendLocalMap::GetLastRemovedKF(FramePtr *f)
    {
        (*f) = last_removed_kf_;
        if (last_removed_kf_)
        {
            last_removed_kf_.reset();
        }
    }

} // namespace mivins
