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

#include <mivins/direct/patch_matcher.h>

#include <cstdlib>
#include <random>
#include <chrono>

#include <mivins/utils/cv_utils.h>
#include <mivins/utils/math_utils.h>
#include <kindr/minimal/angle_axis.h>

#include <mivins/direct/patch_affine_warp.h>
#include <mivins/direct/patch_zmssd_score.h>
#include <mivins/direct/patch_utilities.h>
#include <mivins/common/frame.h>
#include <mivins/common/point.h>
#include <mivins/common/feature_wrapper.h>
#include <mivins/direct/feature_patch_alignment.h>
#include <mivins/common/camera.h>
#include <mivins/common/logging.h>

namespace mivins
{

    PatchMatcher::MatchResult PatchMatcher::FindMatchDirect(
        const Frame &ref_frame,
        const Frame &cur_frame,
        const FeatureWrapper &ref_ftr,
        const FloatType &ref_depth,
        Keypoint &px_cur)
    {
        Eigen::Vector2i pxi = ref_ftr.px.cast<int>() / (1 << ref_ftr.level);
        int boundary = c_half_patch_size + 2;
        if (pxi[0] < boundary || pxi[1] < boundary || pxi[0] >= static_cast<int>(ref_frame.cam()->imageWidth() / (1 << ref_ftr.level)) - boundary || pxi[1] >= static_cast<int>(ref_frame.cam()->imageHeight() / (1 << ref_ftr.level)) - boundary)
            return MatchResult::kFailVisibility;

        // warp affine
        warp::GetWarpAffineMatrix(
            ref_frame.cam_, cur_frame.cam_, ref_ftr.px, ref_ftr.f, ref_depth,
            cur_frame.T_cam_world() * ref_frame.T_world_cam(), ref_ftr.level, &m_affine_cur_ref);
        m_search_level = warp::GetBestSearchLevel(m_affine_cur_ref, ref_frame.img_pyr_.size() - 1);
        /*
  if(pt.normal_set_)
  {
    warp::GetWarpMatrixAffineHomography(
        *ref_ftr.frame->cam_,
        *cur_frame.cam_,
        ref_ftr.px,
        ref_ftr.f,
        ref_ftr.frame->T_f_w_.rotation_matrix()*pt.normal_,
        (pt.pos3d_in_w-ref_ftr.frame->GetCameraPosInWorld()).norm(),
        cur_frame.T_f_w_*ref_ftr.frame->T_f_w_.Inverse(),
        ref_ftr.level,
        m_affine_cur_ref);
  }
  else
*/
        if (m_patch_matcher_options.use_affine_warp_)
        {
            if (!warp::DoWarpAffine(m_affine_cur_ref, ref_frame.img_pyr_[ref_ftr.level], ref_ftr.px,
                                    ref_ftr.level, m_search_level, c_half_patch_size + 1, m_patch_with_border))
                return MatchResult::kFailWarp;
        }
        else
        {
            // pixelwise warp:
            // TODO(zzc): currently using the search level from affine, good enough?
            if (!warp::WarpPixelwise(cur_frame, ref_frame, ref_ftr,
                                     ref_ftr.level, m_search_level, c_half_patch_size + 1, m_patch_with_border))
                return MatchResult::kFailWarp;
        }
        patch_utils::CreatePatchFromPatchWithBorder(
            m_patch_with_border, c_patch_size, m_patch);

        // px_cur should be set
        Keypoint px_scaled(px_cur / (1 << m_search_level));
        Keypoint px_scaled_start(px_scaled);

        //bool success = false;
        if (isEdgelet(ref_ftr.type))
        {
            GradientVector dir_cur(m_affine_cur_ref * ref_ftr.grad);
            dir_cur.normalize();
            if (feature_patch_alignment::Align1D(
                    cur_frame.img_pyr_[m_search_level], dir_cur, m_patch_with_border,
                    m_patch, m_patch_matcher_options.align_max_iter,
                    m_patch_matcher_options.affine_est_offset_, m_patch_matcher_options.affine_est_gain_,
                    &px_scaled, cur_frame.aff, cur_frame.exposure_time, ref_frame.aff, ref_frame.exposure_time, &m_h_inv))
            {
                if ((px_scaled - px_scaled_start).norm() >
                    m_patch_matcher_options.max_patch_diff_ratio * c_patch_size)
                {
                    return MatchResult::kFailTooFar;
                }
                px_cur = px_scaled * (1 << m_search_level);
                // set member variables with results (used in reprojector)
                m_px_cur = px_cur;
                cur_frame.cam()->backProject3(m_px_cur, &m_f_cur);
                m_f_cur.normalize();
                return MatchResult::kSuccess;
            }
        }
        else
        {
            std::vector<Eigen::Vector2f> *last_fail_steps = nullptr;
            bool res = feature_patch_alignment::Align2D(
                cur_frame.img_pyr_[m_search_level], m_patch_with_border, m_patch,
                m_patch_matcher_options.align_max_iter,
                m_patch_matcher_options.affine_est_offset_, m_patch_matcher_options.affine_est_gain_,
                px_scaled, cur_frame.aff, cur_frame.exposure_time, ref_frame.aff, ref_frame.exposure_time, false, last_fail_steps);
            if (res)
            {
                if ((px_scaled - px_scaled_start).norm() >
                    m_patch_matcher_options.max_patch_diff_ratio * c_patch_size)
                {
                    return MatchResult::kFailTooFar;
                }
                px_cur = px_scaled * (1 << m_search_level);
                // set member variables with results (used in reprojector)
                m_px_cur = px_cur;
                cur_frame.cam()->backProject3(m_px_cur, &m_f_cur);
                m_f_cur.normalize();
                return MatchResult::kSuccess;
            }
            else
            {
                VLOG(300) << "NOT CONVERGED: search level " << m_search_level;
            }
        }
        return MatchResult::kFailAlignment;
    }

    PatchMatcher::MatchResult PatchMatcher::FindEpipolarMatchDirect(
        const Frame &ref_frame,
        const Frame &cur_frame,
        const FeatureWrapper &ref_ftr,
        const double d_estimate_inv,
        const double d_min_inv,
        const double d_max_inv,
        double &depth)
    {
        Transformation T_cur_ref = cur_frame.T_f_w_ * ref_frame.T_f_w_.Inverse();
        return FindEpipolarMatchDirect(ref_frame, cur_frame, T_cur_ref, ref_ftr,
                                       d_estimate_inv, d_min_inv, d_max_inv, depth);
    }

    PatchMatcher::MatchResult PatchMatcher::FindEpipolarMatchDirect(
        const Frame &ref_frame,
        const Frame &cur_frame,
        const Transformation &T_cur_ref,
        const FeatureWrapper &ref_ftr,
        const double d_estimate_inv,
        const double d_min_inv,
        const double d_max_inv,
        double &depth)
    {
        int zmssd_best = PatchScore::threshold();

        // Compute start and end of epipolar line in old_kf for match search, on image plane
        const BearingVector A = T_cur_ref.GetRotation().Rotate(ref_ftr.f) / d_min_inv + T_cur_ref.GetPosition();
        const BearingVector B = T_cur_ref.GetRotation().Rotate(ref_ftr.f) / d_max_inv + T_cur_ref.GetPosition();
        Eigen::Vector2d px_A, px_B;
        cur_frame.cam()->project3(A, &px_A);
        cur_frame.cam()->project3(B, &px_B);
        m_epi_image = px_A - px_B;

        // Compute affine warp matrix
        warp::GetWarpAffineMatrix(
            ref_frame.cam_, cur_frame.cam_, ref_ftr.px, ref_ftr.f,
            1.0 / std::max(0.000001, d_estimate_inv), T_cur_ref, ref_ftr.level, &m_affine_cur_ref);

        // feature pre-selection
        m_reject = false;
        if (isEdgelet(ref_ftr.type) && m_patch_matcher_options.epi_search_edgelet_filtering)
        {
            const Eigen::Vector2d grad_cur = (m_affine_cur_ref * ref_ftr.grad).normalized();
            const double cosangle = fabs(grad_cur.dot(m_epi_image.normalized()));
            if (cosangle < m_patch_matcher_options.epi_search_edgelet_max_angle)
            {
                m_reject = true;
                return MatchResult::kFailAngle;
            }
        }

        // prepare for match
        //    - find best search level
        //    - warp the reference patch
        m_search_level = warp::GetBestSearchLevel(m_affine_cur_ref, ref_frame.img_pyr_.size() - 1);
        // length and direction on SEARCH LEVEL
        m_epi_length_pyramid = m_epi_image.norm() / (1 << m_search_level);
        GradientVector epi_dir_image = m_epi_image.normalized();
        if (!warp::DoWarpAffine(m_affine_cur_ref, ref_frame.img_pyr_[ref_ftr.level], ref_ftr.px,
                                ref_ftr.level, m_search_level, c_half_patch_size + 1, m_patch_with_border))
            return MatchResult::kFailWarp;
        patch_utils::CreatePatchFromPatchWithBorder(
            m_patch_with_border, c_patch_size, m_patch);

        // Case 1: direct search locally if the epipolar line is too short
        if (m_epi_length_pyramid < 2.0)
        {
            m_px_cur = (px_A + px_B) / 2.0;
            MatchResult res = FindLocalMatch(cur_frame, ref_frame, epi_dir_image, m_search_level, m_px_cur);
            if (res != MatchResult::kSuccess)
                return res;
            cur_frame.cam()->backProject3(m_px_cur, &m_f_cur);
            m_f_cur.normalize();
            return matcher_utils::DepthFromTriangulation(T_cur_ref, ref_ftr.f, m_f_cur, &depth);
        }

        // Case 2: search along the epipolar line for the best match
        PatchScore patch_score(m_patch); // precompute for reference patch
        BearingVector C = T_cur_ref.GetRotation().Rotate(ref_ftr.f) / d_estimate_inv + T_cur_ref.GetPosition();
        ScanEpipolarLine(cur_frame, A, B, C, patch_score, m_search_level, &m_px_cur, &zmssd_best);

        // check if the best match is good enough
        if (zmssd_best < PatchScore::threshold())
        {
            if (m_patch_matcher_options.subpix_refinement)
            {
                MatchResult res = FindLocalMatch(cur_frame, ref_frame, epi_dir_image, m_search_level, m_px_cur);
                if (res != MatchResult::kSuccess)
                    return res;
            }
            cur_frame.cam()->backProject3(m_px_cur, &m_f_cur);
            m_f_cur.normalize();
            return matcher_utils::DepthFromTriangulation(T_cur_ref, ref_ftr.f, m_f_cur, &depth);
        }
        else
            return MatchResult::kFailScore;
    }

    std::string PatchMatcher::GetResultString(const PatchMatcher::MatchResult &result)
    {
        std::string result_str = "success";
        switch (result)
        {
        case MatchResult::kFailScore:
            result_str = "fail score";
            break;
        case MatchResult::kFailTriangulation:
            result_str = "fail triangulation";
            break;
        case MatchResult::kFailVisibility:
            result_str = "fail visibility";
            break;
        case MatchResult::kFailWarp:
            result_str = "fail warp";
            break;
        case MatchResult::kFailAlignment:
            result_str = "fail alignment";
            break;
        case MatchResult::kFailRange:
            result_str = "fail range";
            break;
        case MatchResult::kFailAngle:
            result_str = "fail angle";
            break;
        case MatchResult::kFailCloseView:
            result_str = "fail close view";
            break;
        case MatchResult::kFailLock:
            result_str = "fail lock";
            break;
        default:
            result_str = "unknown";
        }
        return result_str;
    }

    PatchMatcher::MatchResult PatchMatcher::FindLocalMatch(
        const Frame &cur_frame,
        const Frame &ref_frame,
        const Eigen::Ref<GradientVector> &direction,
        const int patch_level,
        Keypoint &px_cur)
    {
        Keypoint px_scaled(px_cur / (1 << patch_level));
        bool res;
        if (m_patch_matcher_options.align_1d)
            res = feature_patch_alignment::Align1D(
                cur_frame.img_pyr_[patch_level], direction,
                m_patch_with_border, m_patch,
                m_patch_matcher_options.align_max_iter,
                m_patch_matcher_options.affine_est_offset_, m_patch_matcher_options.affine_est_gain_,
                &px_scaled, cur_frame.aff, cur_frame.exposure_time, ref_frame.aff, ref_frame.exposure_time, &m_h_inv);
        else
            res = feature_patch_alignment::Align2D(
                cur_frame.img_pyr_[patch_level], m_patch_with_border, m_patch,
                m_patch_matcher_options.align_max_iter,
                m_patch_matcher_options.affine_est_offset_, m_patch_matcher_options.affine_est_gain_,
                px_scaled, cur_frame.aff, cur_frame.exposure_time, ref_frame.aff, ref_frame.exposure_time);

        if (!res)
            return MatchResult::kFailAlignment;

        px_cur = px_scaled * (1 << patch_level);
        return MatchResult::kSuccess;
    }

    bool PatchMatcher::UpdateZMSSD(
        const Frame &frame,
        const Eigen::Vector2i &pxi,
        const int patch_level,
        const PatchScore &patch_score,
        int *zmssd_best)
    {
        // TODO interpolation would probably be a good idea
        uint8_t *cur_patch_ptr = frame.img_pyr_[patch_level].data + (pxi[1] - c_half_patch_size) * frame.img_pyr_[patch_level].step + (pxi[0] - c_half_patch_size);
        int zmssd = patch_score.computeScore(cur_patch_ptr, frame.img_pyr_[patch_level].step);

        if (zmssd < *zmssd_best)
        {
            *zmssd_best = zmssd;
            return true;
        }
        else
            return false;
    }

    bool PatchMatcher::IsPatchWithinImage(
        const Frame &frame,
        const Eigen::Vector2i &pxi,
        const int patch_level)
    {
        if (frame.cam()->GetType() == Camera::Type::kMei)
        {
            int64_t h_dist = static_cast<int64_t>(pxi[1]) -
                             static_cast<int64_t>(frame.cam()->imageHeight() / (1 << (patch_level + 1)));
            int64_t w_dist = static_cast<int64_t>(pxi[0]) -
                             static_cast<int64_t>(frame.cam()->imageWidth() / (1 << (patch_level + 1)));
            double dist_squared = std::pow(h_dist, 2) + std::pow(w_dist, 2);
            return !(dist_squared > static_cast<double>((static_cast<int>(frame.cam()->imageWidth() / (1 << patch_level)) - c_patch_size) * (static_cast<int>(frame.cam()->imageWidth() / (1 << patch_level)) - c_patch_size) / 4) || pxi[0] < c_patch_size || pxi[1] < c_patch_size || pxi[0] >= (static_cast<int>(frame.cam()->imageWidth() / (1 << patch_level)) - c_patch_size) || pxi[1] >= (static_cast<int>(frame.cam()->imageHeight() / (1 << patch_level)) - c_patch_size));
        }
        else
        {
            return !(pxi[0] < c_patch_size || pxi[1] < c_patch_size || pxi[0] >= (static_cast<int>(frame.cam()->imageWidth() / (1 << patch_level)) - c_patch_size) || pxi[1] >= (static_cast<int>(frame.cam()->imageHeight() / (1 << patch_level)) - c_patch_size));
        }
    }

    void PatchMatcher::ScanEpipolarLine(
        const Frame &frame,
        const Eigen::Vector3d &A,
        const Eigen::Vector3d &B,
        const Eigen::Vector3d &C,
        const PatchScore &patch_score,
        const int patch_level,
        Keypoint *image_best,
        int *zmssd_best)
    {
        if (m_patch_matcher_options.scan_on_unit_sphere)
            ScanEpipolarUnitSphere(frame, A, B, C, patch_score, patch_level, image_best, zmssd_best);
        else
            ScanEpipolarUnitPlane(frame, A, B, C, patch_score, patch_level, image_best, zmssd_best);
    }

    void PatchMatcher::ScanEpipolarUnitPlane(
        const Frame &frame,
        const Eigen::Vector3d &A,
        const Eigen::Vector3d &B,
        const Eigen::Vector3d &C,
        const PatchScore &patch_score,
        const int patch_level,
        Keypoint *image_best,
        int *zmssd_best)
    {
        // if there're too many steps, we only search for a limited range around the center
        //    while keeping the step size small enough to check each pixel on the image plane
        size_t n_steps = m_epi_length_pyramid / 0.7; // one step per pixel
        Eigen::Vector2d step = (vk::project2(A) - vk::project2(B)) / n_steps;
        if (n_steps > m_patch_matcher_options.max_epi_search_steps)
        {
            /* TODO
    printf("WARNING: skip epipolar search: %d evaluations, px_lenght=%f, d_min=%f, d_max=%f.\n",
           n_steps, epi_length_, d_min_inv, d_max_inv);
    */
            n_steps = m_patch_matcher_options.max_epi_search_steps;
        }
        // now we sample along the epipolar line
        Eigen::Vector2d uv_C = vk::project2(C);
        Eigen::Vector2d uv = uv_C;
        Eigen::Vector2d uv_best = uv;
        bool forward = true;
        Eigen::Vector2i last_checked_pxi(0, 0);

        for (size_t i = 0; i < n_steps; ++i, uv += step)
        {
            Eigen::Vector2d px;
            frame.cam()->project3(vk::unproject2d(uv), &px);
            Eigen::Vector2i pxi(px[0] / (1 << patch_level) + 0.5,
                                px[1] / (1 << patch_level) + 0.5); // +0.5 to round to closest int

            if (pxi == last_checked_pxi)
                continue;
            last_checked_pxi = pxi;

            // check if the patch is full within the new frame
            if (!IsPatchWithinImage(frame, pxi, patch_level))
            {
                // change search direction if pixel is out of field of view
                if (forward)
                {
                    // reverse search direction
                    i = n_steps * 0.5;
                    step = -step;
                    uv = uv_C;
                    forward = false;
                    continue;
                }
                else
                    break;
            }

            if (UpdateZMSSD(frame, pxi, patch_level, patch_score, zmssd_best))
                uv_best = uv;

            if (forward && i > n_steps * 0.5)
            {
                // reverse search direction
                step = -step;
                uv = uv_C;
                forward = false;
            }
        }

        // convert uv_best to image coordinates
        Eigen::Vector2d projected;
        frame.cam()->project3(vk::unproject2d(uv_best), &projected);
        *image_best = projected.cast<mivins::FloatType>();
    }

    void PatchMatcher::ScanEpipolarUnitSphere(
        const Frame &frame,
        const Eigen::Vector3d &A,
        const Eigen::Vector3d &B,
        const Eigen::Vector3d &C,
        const PatchScore &patch_score,
        const int patch_level,
        Keypoint *image_best,
        int *zmssd_best)
    {
        size_t n_steps = m_epi_length_pyramid / 0.7; // TODO(zzc): better way of doing this?
        n_steps = n_steps > m_patch_matcher_options.max_epi_search_steps ? m_patch_matcher_options.max_epi_search_steps : n_steps;
        size_t half_steps = n_steps / 2;

        // calculate the step in angle
        Eigen::Vector3d f_A = A.normalized();
        Eigen::Vector3d f_B = B.normalized();
        double step = std::acos(f_A.dot(f_B)) / n_steps;

        // calculate the rotation axis: positive angle -> toward A
        kindr::minimal::AngleAxis rotation_B_to_A;
        rotation_B_to_A.SetAxis((f_B.cross(f_A)).normalized());

        // search around center
        Eigen::Vector3d f_C = C.normalized();
        Eigen::Vector3d f = f_C;
        Eigen::Vector3d f_best = f_C;
        Eigen::Vector2i last_checked_pxi(0, 0);
        for (size_t i = 0; i < n_steps; i++)
        {
            // TODO(zzc): more compact
            // rotation angle w.r.t. f_C
            double angle = 0.0;
            if (i < half_steps) // f_A <-- f_C
                angle = i * step;
            else
                angle = (i - half_steps) * (-step); // f_C --> f_B
            rotation_B_to_A.SetAngle(angle);

            // current sample on unit sphere
            f = rotation_B_to_A.Rotate(f_C);

            // back project to image plane
            Eigen::Vector2d px;
            frame.cam()->project3(f, &px);
            Eigen::Vector2i pxi(px[0] / (1 << patch_level) + 0.5,
                                px[1] / (1 << patch_level) + 0.5); // +0.5 to round to closest int
            if (pxi == last_checked_pxi)
                continue;
            last_checked_pxi = pxi;

            // is within image?
            // TODO(zzc): FIX use visibility check in camera model
            if (!IsPatchWithinImage(frame, pxi, patch_level))
            {
                if (i < half_steps) // f_A <-- f_C to f_C --> f_B
                {
                    i = half_steps;
                    continue;
                }
                else // end of search
                    break;
            }

            // update ZMSSD
            if (UpdateZMSSD(frame, pxi, patch_level, patch_score, zmssd_best))
                f_best = f;
        }

        //backproject to image plane
        Eigen::Vector2d projected;
        frame.cam()->project3(f_best, &projected);
        *image_best = projected.cast<mivins::FloatType>();
    }

    namespace matcher_utils
    {

        PatchMatcher::MatchResult DepthFromTriangulation(
            const Transformation &T_search_ref,
            const Eigen::Vector3d &f_ref,
            const Eigen::Vector3d &f_cur,
            double *depth)
        {
            Eigen::Matrix<double, 3, 2> A;
            A << T_search_ref.GetRotation().Rotate(f_ref), f_cur;
            const Eigen::Matrix2d AtA = A.transpose() * A;
            if (AtA.determinant() < 0.000001)
                return PatchMatcher::MatchResult::kFailTriangulation;
            const Eigen::Vector2d depth2 = -AtA.inverse() * A.transpose() * T_search_ref.GetPosition();
            (*depth) = std::fabs(depth2[0]);
            return PatchMatcher::MatchResult::kSuccess;
        }

    } // namespace matcher_utils
} // namespace mivins
