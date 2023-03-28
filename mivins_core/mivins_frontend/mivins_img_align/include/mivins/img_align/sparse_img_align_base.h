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

#include <mivins/solver/mini_least_squares_solver.h>
#include <mivins/solver/loss_function.h>
#include <mivins/utils/performance_monitor.h>
#include <mivins/common/types.h>
#include <mivins/common/frame.h>

namespace mivins
{

    using Eigen::Matrix;
    using Eigen::Matrix2f;
    using Eigen::Matrix3d;
    using Eigen::Vector2d;
    using Eigen::Vector3d;

    typedef Eigen::Matrix<double, 8, 8> Matrix8d;
    typedef Eigen::Matrix<double, 8, 1> Vector8d;
    typedef Eigen::Matrix<FloatType, 2, 1> Vector2ft;
    typedef Eigen::Matrix<FloatType, 3, 1> Vector3ft;
    typedef Eigen::Matrix<FloatType, 8, 1> Vector8ft;

    struct SparseImgAlignOptions
    {
        int img_align_max_level = 4;
        int img_align_min_level = 1;
        bool img_align_use_distortion_jacobian = false;
        bool img_align_robustification = false;
        double img_align_weight_scale = 10;
        bool img_align_estimate_alpha = false; //illumination_gain
        bool img_align_estimate_beta = false;  //illumination_offset
        bool img_align_estimate_ab = false;    //AffLight
    };

    /// State to be estimated
    struct SparseImgAlignState
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Transformation T_icur_iref; //Relative transformation
        double alpha = 0.0;         //Affine illumination model multiplicative parameter
        double beta = 0.0;          //Affine illumination model additive parameter
                                    // TODO: if we have multiple frames, we should have alpha&beta for every frame individually
    };

    /// Optimize the pose of the frame by minimizing the photometric error of feature patches.
    class SparseImgAlignBase : public mivins::MiniLeastSquaresSolver<8, SparseImgAlignState, SparseImgAlignBase>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef std::shared_ptr<SparseImgAlignBase> Ptr;
        typedef mivins::MiniLeastSquaresSolverOptions SolverOptions;

        SparseImgAlignBase(
            SolverOptions optimization_options,
            SparseImgAlignOptions options);

        virtual size_t Run(const FrameBundle::Ptr &ref_frames,
                           const FrameBundle::Ptr &cur_frames) = 0;

        static SolverOptions GetDefaultSolverOptions();

        void SetWeightedPrior(
            const Transformation &T_cur_ref_prior,
            const double alpha_prior,
            const double beta_prior,
            const double lambda_rot,
            const double lambda_trans,
            const double lambda_alpha,
            const double lambda_beta);

        template <class derived>
        void SetPatchSize(size_t patch_size)
        {
            m_patch_size = patch_size;
            m_border_size = 1;
            m_patch_size_with_border = m_patch_size + 2 * m_border_size;
            m_patch_area = m_patch_size * m_patch_size;
            static_cast<derived *>(this)->SetPatchSizeSideEffects();
        }

        /// The derived class has to specify the necessary modifications (e.g. memory allocation)
        /// in case the patchsize is changed.
        virtual void SetPatchSizeSideEffects() = 0;

        /// Set number of features used for the alignment. If a number is <= 0 all features are used
        /// Specifying a number gives you the control about the performance of the alignment.
        /// Having an alignment running over 600 or 100 features doesn't make that much a difference
        inline void SetMaxNumFeaturesToAlign(int num)
        {
            m_max_num_features = num;
        }

        /// Set inital value for illumination estimation multiplicative parameter
        inline void SetAlphaInitialValue(double alpha_init)
        {
            m_alpha_init = alpha_init;
        }

        /// Set inital value for illumination estimation additive parameter
        inline void SetBetaInitialValue(double beta_init)
        {
            m_beta_init = beta_init;
        }

        /// Set compensation
        inline void SetCompensation(const bool do_compensation)
        {
            m_options.img_align_estimate_alpha = do_compensation;
            m_options.img_align_estimate_beta = do_compensation;
        }

        /// Warp the (cur)rent image such that it aligns with the (ref)erence image
        virtual double EvaluateError(
            const SparseImgAlignState &state,
            HessianMatrix *H,
            GradientVector *g) = 0;

        void Update(
            const SparseImgAlignState &old_model,
            const UpdateVector &dx,
            SparseImgAlignState &new_model);

        void ApplyPrior(const SparseImgAlignState &current_model);

        virtual void FinishIteration() {}

        static constexpr int kJacobianSize = 8;
        static constexpr int kHessianTriagN = 36; // Nr elements of the upper triangular part of the Hessian
        int m_patch_size;
        int m_border_size;
        int m_patch_size_with_border;
        int m_patch_area;

    protected:
        SparseImgAlignOptions m_options;
        FrameBundle::Ptr m_ref_frames;
        FrameBundle::Ptr m_cur_frames;
        int m_level;
        double m_prior_lambda_rot;
        double m_prior_lambda_trans;
        double m_prior_lambda_alpha;
        double m_prior_lambda_beta;
        int m_max_num_features = -1;
        double m_alpha_init = 0.0;
        double m_beta_init = 0.0;
        LossFunctionPtr m_loss_function;
        double m_weight_scale;
    };

} // namespace mivins
