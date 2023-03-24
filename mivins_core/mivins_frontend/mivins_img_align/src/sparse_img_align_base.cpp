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

#include <mivins/img_align/sparse_img_align_base.h>

#include <algorithm>
#include <random> // std::mt19937

#include <opencv2/highgui/highgui.hpp>

#include <mivins/utils/cv_utils.h>
#include <mivins/utils/math_utils.h>

#include <mivins/common/logging.h>
#include <mivins/common/point.h>
#include <mivins/direct/depth_optimization.h>

namespace mivins
{

    SparseImgAlignBase::SparseImgAlignBase(
        SolverOptions optimization_options,
        SparseImgAlignOptions options)
        : mivins::MiniLeastSquaresSolver<8, SparseImgAlignState, SparseImgAlignBase>(optimization_options), m_options(options), m_loss_function(m_options.img_align_robustification ? (new TukeyLossFunction) : nullptr), m_weight_scale(m_options.img_align_weight_scale)
    {
    }

    SparseImgAlignBase::SolverOptions SparseImgAlignBase::GetDefaultSolverOptions()
    {
        SolverOptions options;
        options.strategy = mivins::Strategy::GaussNewton;
        //options.strategy = vk::solver::Strategy::LevenbergMarquardt;
        options.max_iter = 10;
        options.eps = 0.0005;
        return options;
    }

    void SparseImgAlignBase::SetWeightedPrior(
        const Transformation &T_cur_ref_prior,
        const double alpha_prior,
        const double beta_prior,
        const double lambda_rot,
        const double lambda_trans,
        const double lambda_alpha,
        const double lambda_beta)
    {
        m_prior_lambda_rot = lambda_rot;
        m_prior_lambda_trans = lambda_trans;
        m_prior_lambda_alpha = lambda_alpha;
        m_prior_lambda_beta = lambda_beta;
        SparseImgAlignState state;
        state.T_icur_iref = T_cur_ref_prior;
        state.alpha = alpha_prior;
        state.beta = beta_prior;
        SetPrior(state, Matrix8d::Zero());
    }

    void SparseImgAlignBase::Update(
        const SparseImgAlignState &state_old,
        const UpdateVector &dx,
        SparseImgAlignState &state_new)
    {
        state_new.T_icur_iref = state_old.T_icur_iref * Transformation::Exp(-dx.head<6>());
        if (m_options.img_align_estimate_ab)
        {
            state_new.alpha = state_old.alpha - dx(6);
            state_new.beta = state_old.beta - exp(-state_new.alpha) * dx(7);
            //std::cout << "dx:" << dx(6) << "," << dx(7) << ";" << state_new.alpha << "," << state_new.beta << std::endl;
            SVO_DEBUG_STREAM("dx:" << dx(6) << "," << dx(7) << ";" << state_new.alpha << "," << state_new.beta);
        }
        else
        {
            state_new.alpha = (state_old.alpha - dx(6)) / (1.0 + dx(6));
            state_new.beta = (state_old.beta - dx(7)) / (1.0 + dx(6));
            SVO_DEBUG_STREAM("dx:" << dx(6) << "," << dx(7) << ";" << state_new.alpha << "," << state_new.beta);
        }

        // we need to normalize from time to time otherwise rounding errors sum up
        state_new.T_icur_iref.GetRotation().ToImplementation().normalize();
    }

    void SparseImgAlignBase::ApplyPrior(const SparseImgAlignState &state)
    {
        if (m_iter == 0)
        {
            double H_max_diag_trans = 0;
            for (size_t j = 0; j < 3; ++j)
                H_max_diag_trans = std::max(H_max_diag_trans, std::fabs(m_H(j, j)));
            Matrix3d I_trans = Matrix3d::Identity() * m_prior_lambda_trans * H_max_diag_trans;

            double H_max_diag_rot = 0;
            for (size_t j = 3; j < 6; ++j)
                H_max_diag_rot = std::max(H_max_diag_rot, std::fabs(m_H(j, j)));
            Matrix3d I_rot = Matrix3d::Identity() * m_prior_lambda_rot * H_max_diag_rot;

            const double I_alpha = m_prior_lambda_alpha * m_H(6, 6);
            const double I_beta = m_prior_lambda_beta * m_H(7, 7);

            m_info_prior = Matrix8d::Zero();
            m_info_prior.block<3, 3>(0, 0) = I_trans;
            m_info_prior.block<3, 3>(3, 3) = I_rot;
            m_info_prior(6, 6) = I_alpha;
            m_info_prior(7, 7) = I_beta;
        }

        m_H.noalias() += m_info_prior;
        m_g.head<6>() += m_info_prior.block<6, 6>(0, 0) * Transformation::Log(m_state_prior.T_icur_iref.Inverse() * state.T_icur_iref);

        // TODO: this is just a placeholder. derive correct solution
        m_g(6) += m_info_prior(6, 6) * (m_state_prior.alpha - state.alpha);
        m_g(7) += m_info_prior(7, 7) * (m_state_prior.beta - state.beta);
    }

} // namespace mivins
