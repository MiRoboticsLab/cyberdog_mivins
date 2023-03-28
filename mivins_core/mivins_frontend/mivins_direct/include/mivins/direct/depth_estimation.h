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

#include <mivins/common/types.h>
#include <mivins/common/transformation.h>
#include <mivins/solver/mini_least_squares_solver.h>

namespace mivins
{

    class Frame;

    /// Depth estimation by minimizing photometric error.
    class DepthEstimator : public mivins::MiniLeastSquaresSolver<1, double, DepthEstimator>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using SolverOptions = mivins::MiniLeastSquaresSolverOptions;
        using DepthEstimatorState = double;

        static constexpr int c_kPatchHalfsize = 4;
        static constexpr int c_kPatchSize = 2 * c_kPatchHalfsize;
        static constexpr int c_kPatchArea = c_kPatchSize * c_kPatchSize;
        static constexpr int c_kMaxLevel = 4;
        static constexpr int c_kMinLevel = 0;

        DepthEstimator(const SolverOptions &solver_options);
        ~DepthEstimator() = default;

        static SolverOptions getDefaultSolverOptions();

        void Run(
            const FramePtr &cur_frame,
            const FramePtr &ref_frame,
            const int ref_feature_id);

        double EvaluateError(
            const State &params,
            HessianMatrix *H,
            GradientVector *g);

        void Update(
            const State &param_old,
            const UpdateVector &dx,
            State &param_new);

        bool Solve(
            const HessianMatrix &H,
            const GradientVector &g,
            UpdateVector &dx);

        FramePtr m_cur_frame;
        FramePtr m_ref_frame;
        BearingVector m_f_ref;
        Keypoint m_px_ref;
        Transformation m_trans_cur_ref;

        uint8_t m_ref_patch[c_kPatchSize * c_kPatchSize] __attribute__((aligned(16)));
        uint8_t m_ref_patch_with_border[(c_kPatchSize + 2) * (c_kPatchSize + 2)] __attribute__((aligned(16)));

        int m_level = 0;
    };

} // namespace mivins
