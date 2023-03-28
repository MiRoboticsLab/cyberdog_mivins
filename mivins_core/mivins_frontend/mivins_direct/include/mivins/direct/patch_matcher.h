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

#include <array>
#include <mivins/common/types.h>
#include <mivins/common/camera_fwd.h>
#include <mivins/common/transformation.h>

namespace mivins
{

    // forward declarations
    class Point;
    class Frame;
    class FeatureWrapper;

    namespace patch_score
    {
        template <int HALF_PATCH_SIZE>
        class ZMSSD;
    }

    /// Patch-matcher for reprojection-matching and epipolar search in triangulation.
    class PatchMatcher
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        static const int c_half_patch_size = 4;
        static const int c_patch_size = 8;

        typedef mivins::patch_score::ZMSSD<c_half_patch_size> PatchScore;
        typedef std::shared_ptr<PatchMatcher> Ptr;

        struct PatchMatcherOptions
        {
            bool align_1d = false;             //!< in epipolar search: align patch 1D along epipolar line
            int align_max_iter = 10;           //!< number of iterations for aligning the feature patches in gauss newton
            double max_epi_length_optim = 2.0; //!< max length of epipolar line to skip epipolar search and directly go to img align
            size_t max_epi_search_steps = 100; //!< max number of evaluations along epipolar line
            bool subpix_refinement = true;     //!< do gauss newton feature patch alignment after epipolar search
            bool epi_search_edgelet_filtering = true;
            bool scan_on_unit_sphere = true;
            double epi_search_edgelet_max_angle = 0.7;
            bool verbose = false;
            bool use_affine_warp_ = true;
            bool affine_est_offset_ = true;
            bool affine_est_gain_ = false;
            double max_patch_diff_ratio = 2.0;
        } m_patch_matcher_options;

        enum class MatchResult
        {
            kSuccess,
            kFailScore,
            kFailTriangulation,
            kFailVisibility,
            kFailWarp,
            kFailAlignment,
            kFailRange,
            kFailAngle,
            kFailCloseView,
            kFailLock,
            kFailTooFar
        };

        uint8_t m_patch[c_patch_size * c_patch_size] __attribute__((aligned(16)));
        uint8_t m_patch_with_border[(c_patch_size + 2) * (c_patch_size + 2)] __attribute__((aligned(16)));
        Eigen::Matrix2d m_affine_cur_ref; //!< affine warp matrix
        Eigen::Vector2d m_epi_image;      //!< vector from epipolar start to end on the image plane
        double m_epi_length_pyramid;      //!< length of epipolar line segment in pixels on pyrimid level (only used for epipolar search)
        double m_h_inv;                   //!< hessian of 1d image alignment along epipolar line
        int m_search_level;
        bool m_reject;
        Keypoint m_px_cur;
        BearingVector m_f_cur;

        PatchMatcher() = default;
        ~PatchMatcher() = default;

        /// Find a match by directly applying subpix refinement.
        /// IMPORTANT! This function assumes that px_cur is already set to an estimate that is within ~2-3 pixel of the final result!
        MatchResult FindMatchDirect(
            const Frame &ref_frame,
            const Frame &cur_frame,
            const FeatureWrapper &ref_ftr,
            const FloatType &ref_depth,
            Keypoint &px_cur);

        /// Find a match by searching along the epipolar line without using any features.
        MatchResult FindEpipolarMatchDirect(
            const Frame &ref_frame,
            const Frame &cur_frame,
            const FeatureWrapper &ref_ftr,
            const double d_estimate_inv,
            const double d_min_inv,
            const double d_max_inv,
            double &depth);

        MatchResult FindEpipolarMatchDirect(
            const Frame &ref_frame,
            const Frame &cur_frame,
            const Transformation &T_cur_ref,
            const FeatureWrapper &ref_ftr,
            const double d_estimate_inv,
            const double d_min_inv,
            const double d_max_inv,
            double &depth);
        /// search epipolar line between A~C~B for the best match with respect to patch score
        /// the search is done on patch_level, returns image coordinates and best ZMSSD
        void ScanEpipolarLine(
            const Frame &frame,
            const Eigen::Vector3d &A,
            const Eigen::Vector3d &B,
            const Eigen::Vector3d &C,
            const PatchScore &patch_score,
            const int patch_level,
            Keypoint *image_best,
            int *zmssd_best);

        static std::string GetResultString(const PatchMatcher::MatchResult &result);

    private:
        // TODO(zzc): perhaps some of these should be inline
        /// local optimization for m_patch and m_patch_with_border in *frame* around *px_cur*(image)
        MatchResult FindLocalMatch(
            const Frame &cur_frame,
            const Frame &ref_frame,
            const Eigen::Ref<GradientVector> &direction,
            const int patch_level,
            Keypoint &px_cur);

        /// update best zmssd, if a better one is find, return true
        bool UpdateZMSSD(
            const Frame &frame,
            const Eigen::Vector2i &pxi,
            const int patch_level,
            const PatchScore &patch_score,
            int *zmssd_best);

        /// check is patch is fully within image
        bool IsPatchWithinImage(
            const Frame &frame,
            const Eigen::Vector2i &pxi,
            const int patch_level);

        /// search along the epipolar line on image plane
        /// we sample on the UNIT PLANE and check corresponding patches on the image plane
        void ScanEpipolarUnitPlane(const Frame &frame,
                                   const Eigen::Vector3d &A,
                                   const Eigen::Vector3d &B,
                                   const Eigen::Vector3d &C,
                                   const PatchScore &patch_score,
                                   const int patch_level,
                                   Keypoint *image_best,
                                   int *zmssd_best);

        /// search along the epipolar line on image plane
        /// we sample on the UNIT SPHERE and check corresponding patches on the image plane
        void ScanEpipolarUnitSphere(const Frame &frame,
                                    const Eigen::Vector3d &A,
                                    const Eigen::Vector3d &B,
                                    const Eigen::Vector3d &C,
                                    const PatchScore &patch_score,
                                    const int patch_level,
                                    Keypoint *image_best,
                                    int *zmssd_best);
    };

    namespace matcher_utils
    {

        /// calculate feature point depth
        PatchMatcher::MatchResult DepthFromTriangulation(
            const Transformation &T_search_ref,
            const Eigen::Vector3d &f_ref,
            const Eigen::Vector3d &f_cur,
            double *depth);

        /// returns image patch around px (used for testing)
        void CreatePatchWithBorderNoWarp(
            const cv::Mat &img,
            const Eigen::Vector2i &px,
            const int halfpatch_size_without_border,
            uint8_t *patch_with_border);

        /// returns a patch that is 2px smaller in both dimensions.
        void CreatePatchFromPatchWithBorder(
            const uint8_t *const patch_with_border,
            const int patch_size,
            uint8_t *patch);

    } // namespace matcher_utils
} // namespace mivins
