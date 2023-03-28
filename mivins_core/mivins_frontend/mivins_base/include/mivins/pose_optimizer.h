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
#include <mivins/common/frame.h>
#include <mivins/mivins_global_types.h>
#include <mivins/solver/mini_least_squares_solver.h>
#include <mivins/solver/loss_function.h>

namespace mivins
{

    class PoseOptimizer : public mivins::MiniLeastSquaresSolver<6, Transformation, PoseOptimizer>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using SigmaComputer = MedianSigmaComputer;
        using LossFunction = TukeyLossFunction;
        using SolverOptions = mivins::MiniLeastSquaresSolverOptions;
        typedef std::shared_ptr<PoseOptimizer> Ptr;
        typedef Matrix<double, 6, 6> Matrix6d;
        typedef Matrix<double, 2, 6> Matrix26d;
        typedef Matrix<double, 3, 6> Matrix36d;
        typedef Matrix<double, 1, 6> Matrix16d;
        typedef Matrix<double, 6, 1> Vector6d;
        enum class ErrorType
        {
            kUnitPlane,
            kBearingVectorDiff,
            kImagePlane
        };

        struct Statistics
        {
            double reproj_error_after;
            double reproj_error_before;
            Statistics()
                : reproj_error_after(0.0), reproj_error_before(0.0)
            {
            }
        } stats_;

        PoseOptimizer(SolverOptions solver_options);
        virtual ~PoseOptimizer() = default;

        static SolverOptions getDefaultSolverOptions();

        /// Optimize frame pose (frame->T_f_w_) by minimizing the reprojection
        /// error w.r.t. to its features.
        size_t run(const FrameBundle::Ptr &frame, double reproj_thresh);

        void SetRotationPrior(const Quaternion &R_frame_world, double lambda);

        inline size_t IterCount() const
        {
            return m_iter;
        }

        inline void SetErrorType(ErrorType type)
        {
            err_type_ = type;
        }

        inline void InitTracing(const std::string &trace_dir)
        {
            ofs_reproj_errors_.open((trace_dir + "/reproj_errors.txt").c_str());
        }

        //protected:
        FrameBundle::Ptr frame_bundle_;
        double prior_lambda_;
        SigmaComputer sigma_computer_;
        LossFunction loss_function_;
        double measurement_sigma_ = 1.0;
        ErrorType err_type_ = ErrorType::kUnitPlane;
        double focal_length_ = 1.0; /// focal length TODO: should be different for every camera
        std::ofstream ofs_reproj_errors_;

        double EvaluateError(
            const Transformation &T_imu_world,
            HessianMatrix *H,
            GradientVector *g);

        double EvaluateErrorImpl(
            const Transformation &T_imu_world,
            HessianMatrix *H,
            GradientVector *g,
            std::vector<float> *unwhitened_errors);

        void RemoveOutliers(
            const double reproj_err_threshold,
            Frame *frame,
            std::vector<double> *reproj_errors,
            size_t *n_deleted_edges,
            size_t *n_deleted_corners);

        void Update(
            const State &T_frameold_from_world,
            const UpdateVector &dx,
            State &T_framenew_from_world);

        virtual void ApplyPrior(const State &current_model);
    };

    namespace pose_optimizer_utils
    {

        void CalculateFeatureResidualUnitPlane(
            const Eigen::Ref<const BearingVector> &f,
            const Position &xyz_in_world,
            const Transformation &T_imu_world,
            const Transformation &T_cam_imu,
            double measurement_sigma,
            const PoseOptimizer::LossFunction &robust_weight,
            double *unwhitened_error,
            double *chi2_error,
            PoseOptimizer::HessianMatrix *H,
            PoseOptimizer::GradientVector *g);

        void CalculateFeatureResidualImagePlane(
            const Eigen::Ref<const Keypoint> &px,
            const Position &xyz_in_world,
            const Transformation &T_imu_world,
            const Transformation &T_cam_imu,
            const mivins::Camera &cam,
            double measurement_sigma,
            const PoseOptimizer::LossFunction &robust_weight,
            double *unwhitened_error,
            double *chi2_error,
            PoseOptimizer::HessianMatrix *H,
            PoseOptimizer::GradientVector *g);

        void CalculateFeatureResidualBearingVectorDiff(
            const Eigen::Ref<const BearingVector> &f,
            const Position &xyz_in_world,
            const Transformation &T_imu_world,
            const Transformation &T_cam_imu,
            double measurement_sigma,
            const PoseOptimizer::LossFunction &robust_weight,
            double *unwhitened_error,
            double *chi2_error,
            PoseOptimizer::HessianMatrix *H,
            PoseOptimizer::GradientVector *g);

        void CalculateEdgeletResidualUnitPlane(
            const Eigen::Ref<const BearingVector> &f,
            const Position &xyz_in_world,
            const Eigen::Ref<const GradientVector> &grad,
            const Transformation &T_imu_world,
            const Transformation &T_cam_imu,
            double measurement_sigma,
            const PoseOptimizer::LossFunction &robust_weight,
            double *unwhitened_error,
            double *chi2_error,
            PoseOptimizer::HessianMatrix *H,
            PoseOptimizer::GradientVector *g);

        void CalculateEdgeletResidualImagePlane(
            const Eigen::Ref<const Keypoint> &px,
            const Position &xyz_in_world,
            const Eigen::Ref<const GradientVector> &grad,
            const Transformation &T_imu_world,
            const Transformation &T_cam_imu,
            const mivins::Camera &cam,
            double measurement_sigma,
            const PoseOptimizer::LossFunction &robust_weight,
            double *unwhitened_error,
            double *chi2_error,
            PoseOptimizer::HessianMatrix *H,
            PoseOptimizer::GradientVector *g);

        void CalculateEdgeletResidualBearingVectorDiff(
            const Eigen::Ref<const Keypoint> &px,
            const Eigen::Ref<const BearingVector> &f,
            const Position &xyz_in_world,
            const Eigen::Ref<const GradientVector> &grad,
            const Transformation &T_imu_world,
            const Transformation &T_cam_imu,
            const mivins::Camera &cam,
            double measurement_sigma,
            const PoseOptimizer::LossFunction &robust_weight,
            double *unwhitened_error,
            double *chi2_error,
            PoseOptimizer::HessianMatrix *H,
            PoseOptimizer::GradientVector *g);

    } // namespace pose_optimizer_utils

} // namespace mivins
