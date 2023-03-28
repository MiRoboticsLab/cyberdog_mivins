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

#include <mivins/reprojector.h>

#include <set>
#include <random> // std::mt19937
#include <algorithm>
#include <stdexcept>
#include <future>

#include <mivins/common/camera.h>
#include <mivins/common/point.h>
#include <mivins/common/frame.h>
#include <mivins/common/occupancy_grid_2d.h>
#include <mivins/common/logging.h>
#include <mivins/direct/patch_matcher.h>
#include <mivins/direct/depth_optimization.h>
#include <mivins/direct/feature_detector_utilities.h>
#include <mivins/utils/math_utils.h>
#include <mivins/utils/timer.h>

namespace mivins
{

    Reprojector::Reprojector(
        const ReprojectorOptions &options,
        size_t camera_index)
        : options_(options), camera_index_(camera_index)
    {
    }

    void Reprojector::ReprojectFrames(
        const FramePtr &cur_frame,
        const std::vector<FramePtr> &visible_kfs,
        std::vector<PointPtr> &trash_points)
    {
        const size_t max_n_fixed_lm =
            (options_.use_kfs_from_global_map ? options_.max_fixed_landmarks : 0u);
        const size_t max_total_n_features =
            options_.max_n_features_per_frame + max_n_fixed_lm;
        CHECK_GT(options_.max_n_features_per_frame, 0u);
        cur_frame->ResizeFeatureStorage(max_total_n_features);

        // Initialize grid
        if (!grid_)
        {
            grid_.reset(
                new OccupandyGrid2D(
                    options_.cell_size,
                    OccupandyGrid2D::getNCell(cur_frame->cam()->imageWidth(),
                                              options_.cell_size),
                    OccupandyGrid2D::getNCell(cur_frame->cam()->imageHeight(),
                                              options_.cell_size)));
        }
        grid_->reset();
        stats_.reset();

#ifdef SVO_GLOBAL_MAP
        size_t n_fixed_landmarks = 0;
        fixed_lm_stats_.reset();
        if (options_.use_kfs_from_global_map && options_.max_n_global_kfs > 0)
        {
            if (!fixed_landmark_grid_)
            {
                fixed_landmark_grid_.reset(
                    new OccupandyGrid2D(
                        options_.fixed_lm_grid_size,
                        OccupandyGrid2D::getNCell(cur_frame->cam()->imageWidth(),
                                                  options_.fixed_lm_grid_size),
                        OccupandyGrid2D::getNCell(cur_frame->cam()->imageHeight(),
                                                  options_.fixed_lm_grid_size)));
            }
            fixed_landmark_grid_->reset();
            candidates_.clear();
            for (const FramePtr &ref_frame : visible_kfs)
            {
                for (size_t i = 0; i < ref_frame->num_features_; ++i)
                {
                    if (!isFixedLandmark(ref_frame->type_vec_[i]))
                    {
                        continue;
                    }
                    const PointPtr &point = ref_frame->landmark_vec_[i];
                    CHECK(point);

                    if (point->n_failed_reproj_ >= 3 &&
                        point->n_succeeded_reproj_ < 3 * point->n_failed_reproj_)
                    {
                        continue;
                    }

                    if (point->last_projected_kf_id_.at(camera_index_) == cur_frame->id_)
                    {
                        continue;
                    }
                    point->last_projected_kf_id_[camera_index_] = cur_frame->id_;

                    Candidate candidate;
                    if (reprojector_utils::GetCandidate(cur_frame, ref_frame, i, candidate))
                    {
                        candidates_.push_back(candidate);
                    }
                }
            }
            VLOG(10) << "Landmark from global map candidates num: "
                     << candidates_.size() << std::endl;
            using SucFailN = std::array<int, 2>;
            std::vector<std::pair<PointPtr, SucFailN>> old_proj_info;
            for (const auto &c : candidates_)
            {
                const PointPtr &pt = c.ref_frame->landmark_vec_[c.ref_index];
                old_proj_info.emplace_back(
                    std::make_pair(pt,
                                   SucFailN{pt->n_succeeded_reproj_, pt->n_failed_reproj_}));
            }
            reprojector_utils::SortCandidatesByNumObs(candidates_);
            reprojector_utils::MatchCandidates(
                cur_frame, options_.max_fixed_landmarks,
                options_.affine_est_offset, options_.affine_est_gain,
                candidates_, *fixed_landmark_grid_, fixed_lm_stats_,
                options_.seed_sigma2_thresh);
            VLOG(10) << "Reproject landmarks from global map in cam-" << camera_index_
                     << ": trials = " << fixed_lm_stats_.n_trials
                     << ", matches = " << fixed_lm_stats_.n_matches << std::endl;
            if (fixed_lm_stats_.n_matches == 0)
            {
                for (const auto &s_f_info : old_proj_info)
                {
                    s_f_info.first->n_succeeded_reproj_ = s_f_info.second[0];
                    s_f_info.first->n_failed_reproj_ = s_f_info.second[1];
                }
            }
            n_fixed_landmarks = cur_frame->NumFixedLandmarks();
        }
#endif

        // Reproject all map points of the closest N kfs with overlap. We only store
        // in which grid cell the points fall.
        candidates_.clear();
        for (const FramePtr &ref_frame : visible_kfs)
        {
            // Try to reproject each map point that the other KF observes
            for (size_t i = 0; i < ref_frame->num_features_; ++i)
            {
                const FeatureType &type = ref_frame->type_vec_[i];
                if (ref_frame->landmark_vec_[i] == nullptr || type == FeatureType::kOutlier || isMapPoint(ref_frame->type_vec_[i]) || isFixedLandmark(type))
                    continue;

                const PointPtr &point = ref_frame->landmark_vec_[i];

                // first check if the point is valid.
                if (point->n_failed_reproj_ > 10)
                {
                    trash_points.push_back(point);
                    continue;
                }

                // make sure we project a point only once
                if (point->last_projected_kf_id_.at(camera_index_) == cur_frame->GetFrameId())
                    continue;
                point->last_projected_kf_id_[camera_index_] = cur_frame->GetFrameId();

                // make sure the point has at least two observations
                // ignore for the first 5 frames because in the benchmark node we may have
                // initialized some 3d points from the ground-truth.
                if (point->obs_.size() < 2 && options_.remove_unconstrained_points)
                {
                    trash_points.push_back(point);
                    continue;
                }

                // finally, reproject point
                Candidate candidate;
                if (reprojector_utils::GetCandidate(cur_frame, ref_frame, i, candidate))
                    candidates_.push_back(candidate);
            }
        }

        Statistics lm_stats;
        VLOG(10) << "Landmark candidates num: " << candidates_.size() << std::endl;
        reprojector_utils::SortCandidatesByReprojStats(candidates_);
        reprojector_utils::MatchCandidates(
            cur_frame, max_total_n_features,
            options_.affine_est_offset, options_.affine_est_gain,
            candidates_, *grid_, lm_stats, options_.seed_sigma2_thresh);
        VLOG(10) << "After projection: remaining landmark candidates num: "
                 << candidates_.size() << std::endl;
        VLOG(5) << "Reproject landmarks in cam-" << camera_index_
                << ": trials = " << lm_stats.n_trials
                << ", matches = " << lm_stats.n_matches;
        //std::cout << "repro1:" << lm_stats.n_matches << ":" << lm_stats.n_matches << std::endl;
        LOG_DEBUG_STREAM("repro1:"<<lm_stats.n_trials <<":"<<lm_stats.n_matches);
        stats_.add(lm_stats);

        // if we have enough landmarks, we still need to set the grid occupancy
        // to avoid extracting too many new landmarks from the rest candidates
        if (DoesFrameHaveEnoughFeatures(cur_frame))
        {
            reprojector_utils::SetGridCellsOccupied(candidates_, *grid_);
            // We don't return here because we want to
            // check and set all the other grid cells occupied from seeds
        }

        // ---------------------------------------------------------------------------
        // if we don't have enough features, we reproject the converged seeds
        candidates_.clear();
        for (const FramePtr &ref_frame : visible_kfs)
        {
            for (size_t i = 0; i < ref_frame->num_features_; ++i)
            {
                if (IsConvergedCornerEdgeletSeed(ref_frame->type_vec_[i]))
                {
                    Candidate candidate;
                    if (reprojector_utils::GetCandidate(cur_frame, ref_frame, i, candidate))
                        candidates_.push_back(candidate);
                }
            }
        }
        if (DoesFrameHaveEnoughFeatures(cur_frame))
        {
            // In this case we already have enough features but we just want to set the
            // occupancy such that the depth filter doesn't extract too many new features.
            // TODO(cfo): Actually, this should only be done if a new keyframe is selected.
            reprojector_utils::SetGridCellsOccupied(candidates_, *grid_);
            candidates_.clear();
            return;
        }
        Statistics sd_stats;
        VLOG(10) << "Converged seed candidates num " << candidates_.size()
                 << std::endl;
        reprojector_utils::SortCandidatesByReprojStats(candidates_);
        reprojector_utils::MatchCandidates(
            cur_frame, max_total_n_features,
            options_.affine_est_offset, options_.affine_est_gain,
            candidates_, *grid_, sd_stats, options_.seed_sigma2_thresh);
        VLOG(10) << "After projection:"
                 << " remaining converged seed candidates num " << candidates_.size()
                 << std::endl;
        VLOG(5) << "Reproject converged seeds in cam-" << camera_index_
                << ": trials = " << sd_stats.n_trials
                << ", matches = " << sd_stats.n_matches;
        stats_.add(sd_stats);

        if (DoesFrameHaveEnoughFeatures(cur_frame) ||
            !options_.reproject_unconverged_seeds)
        {
            reprojector_utils::SetGridCellsOccupied(candidates_, *grid_);
            candidates_.clear();
            return;
        }

        // ---------------------------------------------------------------------------
        // if we still don't have enough features, reproject the unconverged seeds
        candidates_.clear();
        for (const FramePtr &ref_frame : visible_kfs)
        {
            for (size_t i = 0; i < ref_frame->num_features_; ++i)
            {
                if (IsUnconvergedCornerEdgeletSeed(ref_frame->type_vec_[i]))
                {
                    Candidate candidate;
                    if (reprojector_utils::GetCandidate(cur_frame, ref_frame, i, candidate))
                        candidates_.push_back(candidate);
                }
            }
        }
        Statistics un_sd_stats;
        VLOG(10) << "Unconverged seeds candidates num " << candidates_.size()
                 << std::endl;
        size_t max_allowed_total = max_total_n_features;
        if (options_.max_unconverged_seeds_ratio > 0)
        {
            const double min_lm_seeds_ratio = 1 - options_.max_unconverged_seeds_ratio;
            size_t max_allowed_alternative =
                static_cast<size_t>(
                    (cur_frame->NumTrackedFeatures() + cur_frame->NumFixedLandmarks()) / min_lm_seeds_ratio);
            if (max_allowed_total > max_allowed_alternative)
            {
                max_allowed_total = max_allowed_alternative;
            }
        }
        if (max_allowed_total < options_.min_required_features)
        {
            max_allowed_total = options_.min_required_features;
        }
        VLOG(10) << "Maximum allowed unconverged seeds are " << max_allowed_total;
        reprojector_utils::SortCandidatesByReprojStats(candidates_);
        reprojector_utils::MatchCandidates(
            cur_frame, max_allowed_total,
            options_.affine_est_offset, options_.affine_est_gain,
            candidates_, *grid_, un_sd_stats, options_.seed_sigma2_thresh);
        VLOG(10) << "After projection: "
                    "remaining unconverged seeds candidates num "
                 << candidates_.size()
                 << std::endl;
        VLOG(5) << "Update unconverged seeds in cam-" << camera_index_
                << ": trials = " << un_sd_stats.n_trials
                << ", success = " << un_sd_stats.n_matches;
        stats_.add(un_sd_stats);

        if (DoesFrameHaveEnoughFeatures(cur_frame))
        {
            reprojector_utils::SetGridCellsOccupied(candidates_, *grid_);
        }
        const double un_sd_ratio = (1.0 * un_sd_stats.n_matches) / stats_.n_matches;
        if (un_sd_ratio > 0.2)
        {
            LOG(WARNING) << "More than 20% matches are unconverged seeds: "
                         << un_sd_ratio * 100 << "%";
        }
    }

    namespace reprojector_utils
    {

        void SortCandidatesByReprojStats(
            Reprojector::Candidates &candidates)
        {
            std::sort(candidates.begin(), candidates.end(),
                      [](const Reprojector::Candidate &lhs, const Reprojector::Candidate &rhs)
                      {
                          if (lhs.type > rhs.type || (lhs.type == rhs.type && lhs.n_reproj > rhs.n_reproj) || (lhs.type == rhs.type && lhs.n_reproj == rhs.n_reproj && lhs.score > rhs.score)) // TODO(check)
                              return true;
                          return false;
                      });
        }

        void SortCandidatesByNumObs(
            Reprojector::Candidates &candidates)
        {
            std::sort(candidates.begin(), candidates.end(),
                      [](const Reprojector::Candidate &lhs, const Reprojector::Candidate &rhs)
                      {
                          if (lhs.n_obs > rhs.n_obs || (lhs.n_obs == rhs.n_obs && lhs.n_reproj > rhs.n_reproj) || (lhs.n_obs == rhs.n_obs && lhs.n_reproj == rhs.n_reproj && lhs.score > rhs.score))
                              return true;
                          return false;
                      });
        }

        void MatchCandidates(
            const FramePtr &frame,
            const size_t max_n_features_per_frame,
            const bool affine_est_offset,
            const bool affine_est_gain,
            Reprojector::Candidates &candidates,
            OccupandyGrid2D &grid,
            Reprojector::Statistics &stats,
            const double seed_sigma2_thresh)
        {
            PatchMatcher matcher;
            matcher.m_patch_matcher_options.affine_est_offset_ = affine_est_offset;
            matcher.m_patch_matcher_options.affine_est_gain_ = affine_est_gain;
            int i = 0;
            for (Reprojector::Candidate &candidate : candidates)
            {
                ++i;
                size_t grid_index =
                    grid.getCellIndex(candidate.cur_px.x(), candidate.cur_px.y(), 1);
                if (max_n_features_per_frame > 0 && grid.isOccupied(grid_index))
                {
                    continue;
                }

                ++stats.n_trials;
                FeatureWrapper feature_wrapper = frame->GetEmptyFeatureWrapper();
                if (MatchCandidate(frame, candidate, matcher, feature_wrapper,
                                   seed_sigma2_thresh))
                {
                    ++stats.n_matches;
                    ++frame->num_features_;
                    grid.setOccupied(grid_index);
                    if (max_n_features_per_frame > 0 && frame->num_features_ >= max_n_features_per_frame)
                    {
                        break;
                    }
                }
            }
            candidates.erase(candidates.begin(), candidates.begin() + i);
        }

        bool MatchCandidate(
            const FramePtr &frame,
            Reprojector::Candidate &c,
            PatchMatcher &matcher,
            FeatureWrapper &feature,
            const double seed_sigma2_thresh)
        {
            GradientVector grad_ref;

            // direct matching
            CHECK_NOTNULL(c.ref_frame.get());
            CHECK_LT(c.ref_index, c.ref_frame->num_features_);
            int track_id = -1;
            if (c.ref_frame->landmark_vec_.at(c.ref_index) == nullptr)
            {
                FeatureWrapper ref_ftr = c.ref_frame->GetFeatureWrapper(c.ref_index);
                if (IsConvergedCornerEdgeletSeed(c.type) || isConvergedMapPointSeed(c.type))
                {
                    const FloatType ref_depth = c.ref_frame->GetSeedDepth(c.ref_index);
                    PatchMatcher::MatchResult res =
                        matcher.FindMatchDirect(*c.ref_frame, *frame, ref_ftr, ref_depth,
                                                c.cur_px);
                    if (res != PatchMatcher::MatchResult::kSuccess)
                        return false;
                }
                else if (IsUnconvergedCornerEdgeletSeed(c.type) ||
                         IsUnconvergedMapPointSeed(c.type))
                {
                    if (!depth_optimization_utils::SingleSeedUpdate(
                            *frame, *c.ref_frame, c.ref_index, matcher, seed_sigma2_thresh,
                            false, false))
                    {
                        return false;
                    }
                }
                else
                {
                    CHECK(false) << "Seed type unknown";
                }

                grad_ref = ref_ftr.grad;
                track_id = ref_ftr.track_id;
                feature.seed_ref.keyframe = c.ref_frame;
                feature.seed_ref.seed_id = c.ref_index;
            }
            else
            {
                const PointPtr &point = c.ref_frame->landmark_vec_[c.ref_index];
                FramePtr ref_frame;
                size_t ref_feature_index;
                if (!point->GetCloseViewObs(frame->GetCameraPosInWorld(), ref_frame, ref_feature_index))
                {
                    return false;
                }
                FeatureWrapper ref_ftr = ref_frame->GetFeatureWrapper(ref_feature_index);
                CHECK_NOTNULL(ref_ftr.landmark.get()); // debug
                const FloatType ref_depth = (ref_frame->GetCameraPosInWorld() - ref_ftr.landmark->pos()).norm();
                PatchMatcher::MatchResult res = matcher.FindMatchDirect(
                    *ref_frame, *frame, ref_ftr, ref_depth, c.cur_px);
                if (res != PatchMatcher::MatchResult::kSuccess)
                {
                    point->n_failed_reproj_++;
                    return false; // TODO(cfo): We should return match result and write in statistics.
                }
                point->n_succeeded_reproj_ += 1;
                grad_ref = ref_ftr.grad;
                feature.landmark = point;
                track_id = point->Id();
            }

            // Set edgelet direction, check if is consistent.
            if (isEdgelet(c.type))
            {
                GradientVector g_predicted = (matcher.m_affine_cur_ref * grad_ref).normalized();
                feature.grad = g_predicted;
                /*
    double angle = feature_detector_utils::GetAngleAtPixelUsingHistogram(
          frame->img_pyr_[matcher.m_search_level],
          (matcher.m_px_cur / (1<<matcher.m_search_level)).cast<int>(), 4);
    feature.grad = GradientVector(std::cos(angle), std::sin(angle));

    constexpr double thresh = std::cos(30.0/180.0*3.81);
    if(std::abs(feature.grad.dot(g_predicted)) > thresh)
      return false;
    */
            }

            // Here we add a reference in the feature and frame to the 3D point, the other way
            // round is only done if this frame is selected as keyframe.
            feature.type = c.type;
            feature.px = matcher.m_px_cur;
            feature.f = matcher.m_f_cur;
            feature.level = matcher.m_search_level;
            feature.track_id = track_id;
            feature.score = c.score;
            // TODO: ideally the following should be done via FeatureWrapper
            // This assumes that the feature points to the first free slot
            // i.e., the wrapper is got from GetEmptyFeatureWrapper function
            frame->invmu_sigma2_a_b_vec_.col(frame->NumFeatures()) =
                c.ref_frame->invmu_sigma2_a_b_vec_.col(c.ref_index);
            frame->in_ba_graph_vec_[frame->NumFeatures()] =
                c.ref_frame->in_ba_graph_vec_[c.ref_index];
            return true;
        }

        bool GetCandidate(
            const FramePtr &cur_frame,
            const FramePtr &ref_frame,
            const size_t &ref_index,
            Reprojector::Candidate &candidate)
        {
            Position xyz_world = Position::Zero();
            int n_reproj = 0;
            if (ref_frame->landmark_vec_[ref_index] != nullptr)
            {
                const PointPtr &point = ref_frame->landmark_vec_[ref_index];
                xyz_world = point->pos();
                n_reproj =
                    point->n_succeeded_reproj_ - point->n_failed_reproj_;
            }
            else
            {
                xyz_world = ref_frame->T_world_cam() *
                            ref_frame->GetSeedPosInFrame(ref_index);
            }

            Keypoint px;
            if (!ProjectPointAndCheckVisibility(cur_frame, xyz_world, &px))
            {
                return false;
            }

            candidate = Reprojector::Candidate(
                ref_frame, ref_index, px, n_reproj, ref_frame->score_vec_[ref_index],
                ref_frame->type_vec_[ref_index],
                ref_frame->landmark_vec_[ref_index] ? ref_frame->landmark_vec_[ref_index]->obs_.size() : 0u);

            return true;
        }

        bool ProjectPointAndCheckVisibility(
            const FramePtr &frame,
            const Eigen::Vector3d &xyz,
            Eigen::Vector2d *px)
        {
            CHECK_NOTNULL(px);

            // compute where the point projects and check visibility
            // TODO: insert reasonable clipping distance
            if (!frame->IsVisible(xyz, px))
            {
                return false;
            }
            const Eigen::Vector2i pxi = px->cast<int>();
            constexpr int kPatchSize = 8;
            if (!frame->cam()->isKeypointVisibleWithMargin(pxi, kPatchSize))
                return false;
            return true;
        }

        void SetGridCellsOccupied(
            const Reprojector::Candidates &candidates,
            OccupandyGrid2D &grid)
        {
            for (auto candidate : candidates)
            {
                size_t grid_index =
                    grid.getCellIndex(candidate.cur_px.x(), candidate.cur_px.y(), 1);
                grid.setOccupied(grid_index);
            }
        }

        void ReprojectMapPoints(const FramePtr &frame,
                                const std::vector<FramePtr> &overlap_kfs,
                                const ReprojectorOptions &options,
                                OccupandyGrid2D *grid)
        {
            frame->ResizeFeatureStorage(frame->NumFeatures() + grid->occupancy_.size());
            std::vector<Reprojector::Candidate> candidates;
            grid->reset();
            Reprojector::Statistics stats;
            size_t n_converge_map_point_seed = 0;
            size_t n_map_point = 0;
            size_t n_map_point_seed = 0;
            for (const FramePtr &ref_frame : overlap_kfs)
            {
                // Try to reproject each map point that the other KF observes
                for (size_t i = 0; i < ref_frame->num_features_; ++i)
                {
                    if (ref_frame->type_vec_[i] == FeatureType::kMapPointSeedConverged)
                    {
                        n_converge_map_point_seed++;
                    }
                    else if (ref_frame->type_vec_[i] == FeatureType::kMapPointSeed)
                    {
                        n_map_point_seed++;
                    }
                    else if (ref_frame->type_vec_[i] == FeatureType::kMapPoint)
                    {
                        n_map_point++;
                    }

                    if (ref_frame->type_vec_[i] == FeatureType::kMapPoint ||
                        ref_frame->type_vec_[i] == FeatureType::kMapPointSeedConverged)
                    {
                        if (ref_frame->landmark_vec_[i] != nullptr &&
                            ref_frame->landmark_vec_[i]->n_failed_reproj_ > 10)
                        {
                            continue;
                        }
                        Reprojector::Candidate candidate;
                        if (reprojector_utils::GetCandidate(frame, ref_frame, i, candidate))
                        {
                            candidates.push_back(candidate);
                        }
                    }
                }
            }

            VLOG(5) << "FrontendLocalMap points count: " << std::endl
                    << " - map point: " << n_map_point << std::endl
                    << " - converged map point seed: " << n_converge_map_point_seed << std::endl
                    << " - map point seed: " << n_map_point_seed;

            reprojector_utils::SortCandidatesByReprojStats(candidates);
            reprojector_utils::MatchCandidates(
                frame,
                options.max_n_features_per_frame + options.max_map_features_per_frame,
                options.affine_est_offset, options.affine_est_gain,
                candidates, *grid, stats);

            VLOG(5) << "Reproject landmarks "
                    << ": trials = " << stats.n_trials
                    << ", matches = " << stats.n_matches;
        }

    } // namespace reprojector_utils
} // namespace mivins
