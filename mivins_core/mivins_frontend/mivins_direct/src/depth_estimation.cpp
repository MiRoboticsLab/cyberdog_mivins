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

#include <mivins/direct/depth_estimation.h>
#include <mivins/direct/patch_affine_warp.h>
#include <mivins/direct/patch_utilities.h>
#include <mivins/common/frame.h>
#include <mivins/common/camera.h>
#include <opencv2/highgui/highgui.hpp>

namespace mivins
{

    DepthEstimator::DepthEstimator(const SolverOptions &solver_options)
        : mivins::MiniLeastSquaresSolver<1, double, DepthEstimator>(solver_options)
    {
        ;
    }

    DepthEstimator::SolverOptions DepthEstimator::getDefaultSolverOptions()
    {
        SolverOptions options;
        options.strategy = mivins::Strategy::GaussNewton;
        options.max_iter = 10;
        options.eps = 0.00001;
        return options;
    }

    void DepthEstimator::Run(
        const FramePtr &cur_frame,
        const FramePtr &ref_frame,
        const int ref_feature_id)
    {
        CHECK_GT(ref_frame->invmu_sigma2_a_b_vec_.cols(), ref_feature_id);
        CHECK_GT(ref_frame->f_vec_.cols(), ref_feature_id);
        CHECK_GT(ref_frame->img_pyr_.size(), static_cast<size_t>(c_kMaxLevel));
        CHECK_GT(cur_frame->img_pyr_.size(), static_cast<size_t>(c_kMaxLevel));

        m_cur_frame = cur_frame;
        m_ref_frame = ref_frame;
        m_f_ref = m_ref_frame->f_vec_.col(ref_feature_id);
        m_px_ref = m_ref_frame->px_vec_.col(ref_feature_id);
        m_trans_cur_ref = m_cur_frame->T_cam_world() * m_ref_frame->T_world_cam();

        double state = m_ref_frame->invmu_sigma2_a_b_vec_(0, ref_feature_id);
        for (m_level = c_kMaxLevel; m_level >= c_kMinLevel; --m_level)
        {
            m_mu = 0.1;
            VLOG(100) << "=== Pyramid Level " << m_level << " ===";
            optimize(state);
        }

        m_ref_frame->invmu_sigma2_a_b_vec_(0, ref_feature_id) = state;
    }

    double DepthEstimator::EvaluateError(
        const DepthEstimatorState &inverse_depth,
        HessianMatrix *H,
        GradientVector *g)
    {

        const BearingVector f_cur =
            m_trans_cur_ref.GetRotation().Rotate(m_f_ref) + m_trans_cur_ref.GetPosition() * inverse_depth;
        Keypoint px_cur;
        Eigen::Matrix<double, 2, 3> projection_jacobian;
        m_cur_frame->cam_->project3(f_cur, &px_cur, &projection_jacobian);
        if (!m_cur_frame->cam_->isKeypointVisibleWithMargin(px_cur, (c_kPatchHalfsize + 3) * m_level))
        {
            VLOG(200) << "Depth Estimation: Cur-Patch out of image."
                      << " px_cur_pyr = (" << (px_cur / (1 << m_level)).transpose()
                      << "), img_size = (" << m_cur_frame->img_pyr_[m_level].cols
                      << " x " << m_cur_frame->img_pyr_[m_level].rows << ")";
            return 0.0;
        }

        // Compute warped patch.
        warp::AffineTransformation2 A_cur_ref;
        warp::GetWarpAffineMatrix(
            m_ref_frame->cam(), m_cur_frame->cam(), m_px_ref, m_f_ref,
            1.0 / std::max(0.000001, inverse_depth), m_trans_cur_ref, m_level, &A_cur_ref);
        if (!warp::DoWarpAffine(A_cur_ref, m_ref_frame->img_pyr_[m_level], m_px_ref,
                                m_level, m_level, c_kPatchHalfsize + 1, m_ref_patch_with_border))
        {
            VLOG(200) << "Depth Estimation: Ref-Patch out of image:"
                      << " m_px_refpyr = " << (m_px_ref / (1 << m_level)).transpose()
                      << "), img_size = (" << m_cur_frame->img_pyr_[m_level].cols
                      << " x " << m_cur_frame->img_pyr_[m_level].rows << ")";
            return 0.0;
        }
        patch_utils::CreatePatchFromPatchWithBorder(
            m_ref_patch_with_border, c_kPatchSize, m_ref_patch);

        uint8_t cur_patch_with_border[(c_kPatchSize + 2) * (c_kPatchSize + 2)] __attribute__((aligned(16)));
        Keypoint px_cur_vec = px_cur / (1 << m_level);
        warp::CreatePatchNoWarpInterpolated(
            m_cur_frame->img_pyr_[m_level], px_cur_vec, c_kPatchHalfsize + 1, cur_patch_with_border);

        if (VLOG_IS_ON(200))
        {
            cv::Mat img_cur_rgb(m_cur_frame->img_pyr_[m_level].size(), CV_8UC3);
            cv::cvtColor(m_cur_frame->img_pyr_[m_level], img_cur_rgb, cv::COLOR_GRAY2RGB);
            cv::Mat img_ref_rgb(m_ref_frame->img_pyr_[m_level].size(), CV_8UC3);
            cv::cvtColor(m_ref_frame->img_pyr_[m_level], img_ref_rgb, cv::COLOR_GRAY2RGB);
            const Eigen::Vector2d px_ref_vec = m_px_ref / (1 << m_level);
            cv::rectangle(img_cur_rgb, cv::Rect(px_cur_vec(0), px_cur_vec(1), c_kPatchSize + 3, c_kPatchSize + 3), cv::Scalar(0, 255, 1));
            cv::rectangle(img_ref_rgb, cv::Rect(px_ref_vec(0), px_ref_vec(1), c_kPatchSize + 3, c_kPatchSize + 3), cv::Scalar(0, 255, 1));
            cv::imshow("img_cur_rgb", img_cur_rgb);
            cv::imshow("img_ref_rgb", img_ref_rgb);
            cv::Mat img_cur(c_kPatchSize + 2, c_kPatchSize + 2, CV_8UC1, cur_patch_with_border);
            cv::Mat img_ref(c_kPatchSize + 2, c_kPatchSize + 2, CV_8UC1, m_ref_patch_with_border);
            cv::imshow("patch_cur", img_cur);
            cv::imshow("patch_ref", img_ref);
            cv::waitKey(0);
        }

        double chi2 = 0.0;
        const int patch_step = c_kPatchSize + 2;
        for (int y = 0; y < c_kPatchSize; ++y)
        {
            uint8_t *cur_px = (uint8_t *)cur_patch_with_border + (y + 1) * patch_step + 1;
            uint8_t *ref_px = (uint8_t *)m_ref_patch_with_border + (y + 1) * patch_step + 1;
            for (int x = 0; x < c_kPatchSize; ++x, ++cur_px, ++ref_px)
            {
                Eigen::Vector2d grad(0.5 * (cur_px[1] - cur_px[-1]),
                                     0.5 * (cur_px[patch_step] - cur_px[-patch_step]));
                double residual = *cur_px - *ref_px;
                double J =
                    grad.dot(projection_jacobian * m_trans_cur_ref.GetRotation().Rotate(m_f_ref * (-1.0 / std::pow(inverse_depth, 2))));

                (*H)(0, 0) += J * J;
                (*g)(0) -= J * residual;
                chi2 += residual * residual;
            }
        }

        return chi2;
    }

    void DepthEstimator::Update(
        const DepthEstimatorState &state_old,
        const UpdateVector &dx,
        DepthEstimatorState &state_new)
    {
        state_new = state_old + dx(0);
    }

    bool DepthEstimator::Solve(
        const HessianMatrix &H,
        const GradientVector &g,
        UpdateVector &dx)
    {
        dx(0) = g(0) / H(0, 0);
        return true;
    }

} // namespace mivins
