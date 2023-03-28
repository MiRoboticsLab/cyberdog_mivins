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

#include <mivins/img_align/sparse_img_align_base.h>

#include <mivins/solver/loss_function.h>
#include <mivins/utils/performance_monitor.h>

#include <mivins/common/types.h>
#include <mivins/common/frame.h>

namespace mivins
{

    typedef Eigen::Matrix<FloatType, 2, Eigen::Dynamic, Eigen::ColMajor> UvCache;
    typedef Eigen::Matrix<FloatType, 3, Eigen::Dynamic, Eigen::ColMajor> XyzRefCache;
    typedef Eigen::Matrix<FloatType, 6, Eigen::Dynamic, Eigen::ColMajor> JacobianProjCache;
    typedef Eigen::Matrix<FloatType, 8, Eigen::Dynamic, Eigen::ColMajor> JacobianCache;
    typedef Eigen::Matrix<FloatType, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> ResidualCache;
    typedef Eigen::Matrix<bool, Eigen::Dynamic, 1, Eigen::ColMajor> VisibilityMask;
    typedef Eigen::Matrix<FloatType, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> RefPatchCache;

    /// Optimize the pose of the frame by minimizing the photometric error of feature patches.
    class SparseImgAlign : public SparseImgAlignBase
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef std::shared_ptr<SparseImgAlign> Ptr;

    public:
        SparseImgAlign(
            SolverOptions optimization_options,
            SparseImgAlignOptions options);

        void SetPatchSizeSideEffects()
        {
            // no side effects
        }

        size_t Run(const FrameBundle::Ptr &ref_frames,
                   const FrameBundle::Ptr &cur_frames);

    private:
        bool m_have_cache = false;
        std::vector<std::vector<size_t>> m_fts_vec;
        UvCache m_uv_cache;                      //<! containts feature coordinates in reference image (size 2 x #Patches)
        XyzRefCache m_xyz_ref_cache;             //!< contains 3d feature location in IMU frame (size 3 x #Patches)
        JacobianProjCache m_jacobian_proj_cache; //!< containts 2x6 Jacobians (d proj(X))/(d xi) (size 2 x 6 x #Patches)
        JacobianCache m_jacobian_cache;          //<! contains 1x8 jacobians (pose and illumination model) (size 8 x AreaPatch*#Patches)
        ResidualCache m_residual_cache;          //<! residuals (size AreaPatch x #Patches)
        VisibilityMask m_visibility_mask;        //<! is Patch visible in current image? (size 1 x #Patches)
        RefPatchCache m_ref_patch_cache;         //<! residuals (size AreaPatch x #Patches)

    protected:
        /// Warp the (cur)rent image such that it aligns with the (ref)erence image
        double EvaluateError(
            const SparseImgAlignState &state,
            HessianMatrix *H,
            GradientVector *g);

        void Update(
            const SparseImgAlignState &old_model,
            const UpdateVector &dx,
            SparseImgAlignState &new_model);

        void ApplyPrior(const SparseImgAlignState &current_model);

        virtual void FinishIteration();
    };

    namespace sparse_img_align_utils
    {

        void ExtractFeaturesSubset(
            const Frame &ref_frame,
            const int max_level,
            const int patch_size_wb, // patch size + border (usually border = 2 for gradiant)
            std::vector<size_t> &fts);

        // Fills UvCache (needed for extraction of refpatch extraction at every pyramid level),
        // XyzRefCache (needed at every optimization step for reprojection) and
        // JacobianProjCache (needed at every pyramid level)
        void PrecomputeBaseCaches(
            const Frame &ref_frame,
            const std::vector<size_t> &fts,
            const bool use_pinhole_distortion,
            size_t &feature_counter,
            UvCache &uv_cache,
            XyzRefCache &xyz_ref_cache,
            JacobianProjCache &jacobian_proj_cache);

        // Fills JacobianCache and RefPatchCache at every level and sets have_cache_ to true
        void PrecomputeJacobiansAndRefPatches(
            const FramePtr &ref_frame,
            const UvCache &uv_cache,
            const JacobianProjCache &jacobian_proj_cache,
            const size_t level,
            const int patch_size,
            const size_t nr_features,
            bool estimate_alpha, bool estimate_beta, bool estimate_ab,
            size_t &feature_counter,
            JacobianCache &jacobian_cache,
            RefPatchCache &ref_patch_cache);

        // Fills ResidualCache and VisibilityMask
        void ComputeResidualsOfFrame(
            const FramePtr &cur_frame,
            const size_t level,
            const int patch_size,
            const size_t nr_features,
            const Transformation &T_cur_ref,
            const float alpha,
            const float beta,
            bool estimate_ab,
            const RefPatchCache &ref_patch_cache,
            const XyzRefCache &xyz_ref_cache,
            size_t &feature_counter,
            std::vector<Vector2d> *match_px,
            ResidualCache &residual_cache,
            VisibilityMask &visibility_mask);

        // Compute Hessian and gradient
        FloatType ComputeHessianAndGradient(
            const JacobianCache &jacobian_cache,
            const ResidualCache &residual_cache,
            const VisibilityMask &visibility_mask,
            const float weight_scale,
            const LossFunctionPtr &loss_function,
            SparseImgAlign::HessianMatrix *H,
            SparseImgAlign::GradientVector *g);

        // Experimental. Computes residuals and Hessian at the same time. No significant speedup was observed.
        float ComputeResidualHessianGradient(
            const FramePtr &cur_frame,
            const size_t level,
            const int patch_size,
            const size_t nr_features,
            const Transformation &T_cur_ref,
            const float alpha,
            const float beta,
            const RefPatchCache &ref_patch_cache,
            const XyzRefCache &xyz_ref_cache,
            const JacobianCache &jacobian_cache,
            const float weight_scale,
            const LossFunctionPtr &loss_function,
            SparseImgAlign::HessianMatrix *H,
            SparseImgAlign::GradientVector *g,
            size_t &feature_counter);
    } // namespace sparse_img_align_utils
} // namespace mivins
