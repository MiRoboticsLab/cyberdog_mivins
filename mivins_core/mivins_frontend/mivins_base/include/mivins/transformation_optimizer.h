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

#include <fstream>
#include <iostream>
#include <mivins/mivins_global_types.h>
#include <mivins/solver/mini_least_squares_solver.h>
#include <mivins/solver/loss_function.h>

using Transformation = kindr::minimal::QuatTransformation;
using Quaternion = kindr::minimal::RotationQuaternion;
using PoseDeque = std::deque<Transformation>;

namespace mivins
{
    class TransformationOptimizer : public mivins::MiniLeastSquaresSolver<6, Transformation, TransformationOptimizer>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using SigmaComputer = MedianSigmaComputer;
        using LossFunction = TukeyLossFunction;
        using SolverOptions = mivins::MiniLeastSquaresSolverOptions;
        typedef std::shared_ptr<TransformationOptimizer> Ptr;
        typedef Matrix<double, 6, 6> Matrix6d;
        typedef Matrix<double, 6, 1> Vector6d;

        struct Statistics
        {
            double pose_error_after;
            double pose_error_before;
            Statistics()
                : pose_error_after(0.0), pose_error_before(0.0)
            {
            }
        } stats_;

        TransformationOptimizer(SolverOptions solver_options);
        virtual ~TransformationOptimizer() = default;

        static SolverOptions getDefaultSolverOptions();

        void run(PoseDeque &vio_pose, PoseDeque &tar_pose, Transformation &T, const double outlier_th, double &inlier_error, double &inlier_ratio);

        void SetTranslationPrior(const Eigen::Vector3d &t_prior, double lambda);

        inline size_t IterCount() const
        {
            return m_iter;
        }

        int pose_num_ = 0;
        PoseDeque vio_pose_;
        PoseDeque tar_pose_;
        double prior_lambda_;
        SigmaComputer sigma_computer_;
        LossFunction loss_function_;
        double measurement_sigma_ = 1.0;

        double EvaluateError(
            const Transformation &T,
            HessianMatrix *H,
            GradientVector *g);

        double EvaluateErrorImpl(
            const Transformation &T,
            HessianMatrix *H,
            GradientVector *g,
            std::vector<float> *unwhitened_errors);

        void Update(
            const State &T_old,
            const UpdateVector &dx,
            State &T_new);

        void CheckOutliers(
            const double outlier_th,
            const Transformation &T,
            PoseDeque &vio_pose,
            PoseDeque &tar_pose,
            std::vector<double> *unwhitened_errors,
            double *inlier_error,
            double *inlier_ratio);

        virtual void ApplyPrior(const State &current_model);
    };

    namespace transformation_optimizer_utils
    {
        Eigen::Matrix3d computeRightJacobian(Quaternion &R);
        Eigen::Matrix3d computeRightJacobianInverse(Quaternion &R);

        double cot(double &x);

        void CalculateResidualHessianJacobian(
            const Transformation &T,
            const Transformation &T_vio,
            const Transformation &T_tar,
            double measurement_sigma,
            const TransformationOptimizer::LossFunction &robust_weight,
            double *unwhitened_error,
            double *chi2_error,
            TransformationOptimizer::HessianMatrix *H,
            TransformationOptimizer::GradientVector *g);
    }

} // namespace mivins
