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

#include <stdexcept>
#include <mivins/utils/math_utils.h>
#include <mivins/common/frame.h>
#include <mivins/common/point.h>
#include <mivins/pose_optimizer.h>
#include <mivins/common/seed.h>

namespace mivins
{

    PoseOptimizer::PoseOptimizer(SolverOptions solver_options)
        : mivins::MiniLeastSquaresSolver<6, Transformation, PoseOptimizer>(solver_options)
    {
    }

    PoseOptimizer::SolverOptions PoseOptimizer::getDefaultSolverOptions()
    {
        SolverOptions options;
        options.strategy = mivins::Strategy::GaussNewton;
        options.max_iter = 10;
        options.eps = 0.000001;
        return options;
    }

    void PoseOptimizer::SetRotationPrior(const Quaternion &R_curimu_world, double lambda)
    {
        Transformation T_curimu_world_prior(R_curimu_world, Vector3d::Zero());
        Matrix6d Information = Matrix6d::Zero();
        Information.bottomRightCorner<3, 3>() = Matrix3d::Identity();
        prior_lambda_ = lambda;
        SetPrior(T_curimu_world_prior, Information);
    }

    size_t PoseOptimizer::run(const FrameBundle::Ptr &frame_bundle, double reproj_thresh_px)
    {
        CHECK(!frame_bundle->empty()) << "PoseOptimizer: FrameBundle is empty";
        //CHECK_GT(frame_bundle->NumFeatures(), 0u) << "PoseOptimizer: No features in frames";

        if(frame_bundle->NumFeatures() < 10)
            return 0;

        focal_length_ = frame_bundle->at(0)->GetErrorMultiplier();
        frame_bundle_ = frame_bundle;
        Transformation T_imu_world = frame_bundle->at(0)->T_imu_world();

        // Check the scale of the errors.
        std::vector<float> start_errors;
        EvaluateErrorImpl(T_imu_world, nullptr, nullptr, &start_errors);
        measurement_sigma_ = sigma_computer_.Compute(start_errors);
        VLOG(5) << "Initial measurement sigma:" << measurement_sigma_;

        // Run Gauss Newton optimization.
        optimize(T_imu_world);

        for (const FramePtr &frame : frame_bundle->frames_)
        {
            frame->T_f_w_ = frame->T_cam_imu() * T_imu_world;
        }

        // Remove Measurements with too large reprojection error
        size_t n_deleted_edges = 0, n_deleted_corners = 0;
        std::vector<double> final_errors;
        for (const FramePtr &f : frame_bundle->frames_)
        {
            RemoveOutliers(reproj_thresh_px, f.get(),
                           &final_errors, &n_deleted_edges, &n_deleted_corners);
        }

        VLOG(5) << "PoseOptimzer: drop " << n_deleted_corners << " corner outliers and "
                << n_deleted_edges << " edgelet outliers out of "
                << m_meas_size << " measurements.";

        // TODO(zzc): for bearing vector difference, can we also save errors in pixels?
        // save statistics
        double error_scale = (err_type_ == ErrorType::kUnitPlane) ? focal_length_ : 1.0;
        stats_.reproj_error_before = vk::getMedian<float>(start_errors) * error_scale;
        stats_.reproj_error_after = vk::getMedian<double>(final_errors) * error_scale;

        // trace to file
        if (ofs_reproj_errors_.is_open())
        {
            for (auto i : start_errors)
                ofs_reproj_errors_ << i * error_scale << ", ";
            ofs_reproj_errors_ << std::endl;
            for (auto i : final_errors)
                ofs_reproj_errors_ << i * error_scale << ", ";
            ofs_reproj_errors_ << std::endl;
        }

        return m_meas_size - n_deleted_edges - n_deleted_corners;
    }

    double PoseOptimizer::EvaluateError(
        const Transformation &T_imu_world,
        HessianMatrix *H,
        GradientVector *g)
    {
        return EvaluateErrorImpl(T_imu_world, H, g, nullptr);
    }

    double PoseOptimizer::EvaluateErrorImpl(
        const Transformation &T_imu_world,
        HessianMatrix *H,
        GradientVector *g,
        std::vector<float> *unwhitened_errors)
    {
        double chi2_error_sum = 0.0;

        // compute the weights on the first iteration
        if (unwhitened_errors)
            unwhitened_errors->reserve(frame_bundle_->NumFeatures());

        for (const FramePtr &frame : frame_bundle_->frames_)
        {
            const Transformation T_cam_imu = frame->T_cam_imu();

            // compute residual and update normal equation
            for (size_t i = 0; i < frame->num_features_; ++i)
            {
                Position xyz_world;
                if (frame->landmark_vec_[i] != nullptr)
                {
                    xyz_world = frame->landmark_vec_[i]->pos3d_in_w;
                }
                else if (isCornerEdgeletSeed(frame->type_vec_[i]))
                {
                    const SeedRef &ref = frame->seed_ref_vec_[i];
                    xyz_world = ref.keyframe->T_world_cam() * ref.keyframe->GetSeedPosInFrame(ref.seed_id);
                }
                else
                    continue;

                const int scale = (1 << frame->level_vec_(i));
                double unwhitened_error, chi2_error;
                double measurement_sigma = measurement_sigma_ * scale;
                if (isEdgelet(frame->type_vec_[i]))
                {
                    // Edgelets should have less weight than corners.
                    constexpr double kEdgeletSigmaExtraFactor = 2.0;
                    measurement_sigma *= kEdgeletSigmaExtraFactor;
                    if (err_type_ == ErrorType::kUnitPlane)
                        pose_optimizer_utils::CalculateEdgeletResidualUnitPlane(
                            frame->f_vec_.col(i), xyz_world, frame->grad_vec_.col(i),
                            T_imu_world, T_cam_imu, measurement_sigma, loss_function_,
                            &unwhitened_error, &chi2_error, H, g);
                    else if (err_type_ == ErrorType::kImagePlane)
                        pose_optimizer_utils::CalculateEdgeletResidualImagePlane(
                            frame->px_vec_.col(i), xyz_world, frame->grad_vec_.col(i),
                            T_imu_world, T_cam_imu, *frame->cam(), measurement_sigma,
                            loss_function_, &unwhitened_error, &chi2_error, H, g);
                    else if (err_type_ == ErrorType::kBearingVectorDiff)
                        pose_optimizer_utils::CalculateEdgeletResidualBearingVectorDiff(
                            frame->px_vec_.col(i), frame->f_vec_.col(i), xyz_world, frame->grad_vec_.col(i),
                            T_imu_world, T_cam_imu, *frame->cam(), measurement_sigma,
                            loss_function_, &unwhitened_error, &chi2_error, H, g);
                }
                else
                {
                    if (err_type_ == ErrorType::kUnitPlane)
                        pose_optimizer_utils::CalculateFeatureResidualUnitPlane(
                            frame->f_vec_.col(i), xyz_world, T_imu_world, T_cam_imu,
                            measurement_sigma, loss_function_,
                            &unwhitened_error, &chi2_error, H, g);
                    else if (err_type_ == ErrorType::kImagePlane)
                    {
                        pose_optimizer_utils::CalculateFeatureResidualImagePlane(
                            frame->px_vec_.col(i), xyz_world, T_imu_world, T_cam_imu,
                            *frame->cam(), measurement_sigma, loss_function_,
                            &unwhitened_error, &chi2_error, H, g);
                    }
                    else if (err_type_ == ErrorType::kBearingVectorDiff)
                    {
                        pose_optimizer_utils::CalculateFeatureResidualBearingVectorDiff(
                            frame->f_vec_.col(i), xyz_world, T_imu_world, T_cam_imu,
                            measurement_sigma, loss_function_,
                            &unwhitened_error, &chi2_error, H, g);
                    }
                }
                if (unwhitened_errors)
                {
                    CHECK_GE(unwhitened_error, 0.0);
                    unwhitened_errors->push_back(unwhitened_error / scale);
                }
                chi2_error_sum += chi2_error;
                ++m_meas_size;
            } // for each feature
        }     // for each frame

        return chi2_error_sum;
    }

    void PoseOptimizer::RemoveOutliers(
        const double reproj_err_threshold,
        Frame *frame,
        std::vector<double> *reproj_errors,
        std::size_t *n_deleted_edges,
        std::size_t *n_deleted_corners)
    {
        CHECK_NOTNULL(frame);
        CHECK_NOTNULL(reproj_errors);
        CHECK_NOTNULL(n_deleted_edges);
        CHECK_NOTNULL(n_deleted_corners);

        // calculate threhold for once
        static double threshold_unitplane = reproj_err_threshold / focal_length_;
        static double threshold_bearing_diff = std::fabs(2 * std::sin(0.5 * frame->GetAngleError(reproj_err_threshold)));

        double outlier_threshold = reproj_err_threshold; // image error
        if (err_type_ == ErrorType::kUnitPlane)
            outlier_threshold = threshold_unitplane;
        else if (err_type_ == ErrorType::kBearingVectorDiff)
            outlier_threshold = threshold_bearing_diff;

        reproj_errors->reserve(frame->num_features_);
        for (size_t i = 0; i < frame->num_features_; ++i)
        {
            Position xyz_world;
            if (frame->landmark_vec_[i] != nullptr)
            {
                xyz_world = frame->landmark_vec_[i]->pos3d_in_w;
            }
            else if (isCornerEdgeletSeed(frame->type_vec_[i]))
            {
                const SeedRef &ref = frame->seed_ref_vec_[i];
                xyz_world = ref.keyframe->T_world_cam() * ref.keyframe->GetSeedPosInFrame(ref.seed_id);
            }
            else
                continue;

            // calculate residual according to different feature type and residual
            Transformation T_imu_world = frame->T_imu_world();
            Transformation T_cam_imu = frame->T_cam_imu();
            double unwhitened_error, chi2_error;
            if (isEdgelet(frame->type_vec_[i]))
            {
                if (err_type_ == ErrorType::kUnitPlane)
                {
                    pose_optimizer_utils::CalculateEdgeletResidualUnitPlane(
                        frame->f_vec_.col(i), xyz_world, frame->grad_vec_.col(i),
                        T_imu_world, T_cam_imu,
                        0.0, loss_function_, &unwhitened_error, &chi2_error, nullptr, nullptr);
                }
                else if (err_type_ == ErrorType::kBearingVectorDiff)
                {
                    pose_optimizer_utils::CalculateEdgeletResidualBearingVectorDiff(
                        frame->px_vec_.col(i), frame->f_vec_.col(i), xyz_world, frame->grad_vec_.col(i),
                        T_imu_world, T_cam_imu, *frame->cam(),
                        0.0, loss_function_, &unwhitened_error, &chi2_error, nullptr, nullptr);
                }
                else if (err_type_ == ErrorType::kImagePlane)
                {
                    pose_optimizer_utils::CalculateEdgeletResidualImagePlane(
                        frame->px_vec_.col(i), xyz_world, frame->grad_vec_.col(i),
                        T_imu_world, T_cam_imu, *frame->cam(),
                        0.0, loss_function_, &unwhitened_error, &chi2_error, nullptr, nullptr);
                }
            }
            else
            {
                if (err_type_ == ErrorType::kUnitPlane)
                {
                    pose_optimizer_utils::CalculateFeatureResidualUnitPlane(
                        frame->f_vec_.col(i), xyz_world, T_imu_world, T_cam_imu,
                        0.0, loss_function_, &unwhitened_error, &chi2_error, nullptr, nullptr);
                }
                else if (err_type_ == ErrorType::kBearingVectorDiff)
                {
                    pose_optimizer_utils::CalculateFeatureResidualBearingVectorDiff(
                        frame->f_vec_.col(i), xyz_world, T_imu_world, T_cam_imu,
                        0.0, loss_function_, &unwhitened_error, &chi2_error, nullptr, nullptr);
                }
                else if (err_type_ == ErrorType::kImagePlane)
                {
                    pose_optimizer_utils::CalculateFeatureResidualImagePlane(
                        frame->px_vec_.col(i), xyz_world, T_imu_world, T_cam_imu, *frame->cam(),
                        0.0, loss_function_, &unwhitened_error, &chi2_error, nullptr, nullptr);
                }
            }
            unwhitened_error *= 1.0 / (1 << frame->level_vec_(i));
            reproj_errors->push_back(unwhitened_error);
            if (std::fabs(unwhitened_error) > outlier_threshold)
            {
                if (isEdgelet(frame->type_vec_[i]))
                    ++(*n_deleted_edges);
                else
                    ++(*n_deleted_corners);

                frame->type_vec_[i] = FeatureType::kOutlier;
                frame->seed_ref_vec_[i].keyframe.reset();
                frame->landmark_vec_[i] = nullptr; // delete landmark observation
            }
        }
    }

    void PoseOptimizer::Update(
        const State &T_imuold_world,
        const UpdateVector &dx,
        State &T_imunew_world)
    {
        T_imunew_world = Transformation::Exp(dx) * T_imuold_world;

        // we need to normalize from time to time, otherwise rounding errors sum up
        T_imunew_world.GetRotation().ToImplementation().normalize();
    }

    void PoseOptimizer::ApplyPrior(const State &T_cur_from_world)
    {
        if (m_iter == 0)
        {
            m_info_prior = Matrix6d::Zero();
            m_info_prior.bottomRightCorner<3, 3>() = Matrix3d::Identity();

            double H_max_diag = 0;
            //double tau = 1e-4;
            for (size_t j = 3; j < 6; ++j)
                H_max_diag = std::max(H_max_diag, std::fabs(m_H(j, j)));
            m_info_prior *= H_max_diag * prior_lambda_;
            //std::cout << "prior_lambda (pose_opt): " << prior_lambda_ << std::endl;
            //std::cout << H_max_diag << std::endl;
            if (m_solver_options.verbose)
            {
                std::cout << "applying rotation prior, I = " << H_max_diag * prior_lambda_ << std::endl;
            }
        }

        m_H.noalias() += m_info_prior;
        //Jres_.noalias() += m_info_prior*Transformation::log(m_state_prior*T_cur_from_world.inverse());
        m_g.noalias() -= m_info_prior * Transformation::Log(T_cur_from_world * m_state_prior.Inverse());
        //std::cout << "information matrix = " << m_info_prior << std::endl;
    }

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
            PoseOptimizer::GradientVector *g)
        {
            const Vector3d xyz_in_imu(T_imu_world * xyz_in_world);
            const Vector3d xyz_in_cam(T_cam_imu * xyz_in_imu);

            // Prediction error.
            Eigen::Vector2d e = vk::project2(f) - vk::project2(xyz_in_cam);
            if (unwhitened_error)
                *unwhitened_error = e.norm();

            // Whiten error: R*e, where R is the square root of information matrix (1/sigma).
            double R = 1.0 / measurement_sigma;
            e *= R;

            // M-estimator weighting
            double weight = robust_weight.Weight(e.norm());

            // Compute log-likelihood : 1/(2*sigma^2)*(z-h(x))^2 = 1/2*e'R'*R*e
            *chi2_error = 0.5 * e.squaredNorm() * weight;

            if (H && g)
            {
                // compute jacobian
                PoseOptimizer::Matrix26d J_proj;
                Frame::Jacobian_xyz2uv_imu(T_cam_imu, xyz_in_imu, J_proj);
                J_proj *= R;
                H->noalias() += J_proj.transpose() * J_proj * weight;
                g->noalias() -= J_proj.transpose() * e * weight;
            }
        }

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
            PoseOptimizer::GradientVector *g)
        {
            const Vector3d xyz_in_imu(T_imu_world * xyz_in_world);
            const Vector3d xyz_in_cam(T_cam_imu * xyz_in_imu);

            // Prediction error.
            Eigen::Matrix<double, 2, 3> J_cam;
            Keypoint px_est;
            cam.project3(xyz_in_cam, &px_est, &J_cam);
            Eigen::Vector2d e = px - px_est;
            if (unwhitened_error)
                *unwhitened_error = e.norm();

            // Whiten error: R*e, where R is the square root of information matrix (1/sigma).
            double R = 1.0 / measurement_sigma;
            e *= R;

            // M-estimator weighting
            double weight = robust_weight.Weight(e.norm());

            // Compute log-likelihood : 1/(2*sigma^2)*(z-h(x))^2 = 1/2*e'R'*R*e
            *chi2_error = 0.5 * e.squaredNorm() * weight;

            if (H && g)
            {
                // compute jacobian
                PoseOptimizer::Matrix26d J_proj;
                Frame::Jacobian_xyz2img_imu(T_cam_imu, xyz_in_imu, J_cam, J_proj);
                J_proj = (-1.0) * J_proj;
                J_proj *= R;
                H->noalias() += J_proj.transpose() * J_proj * weight;
                g->noalias() -= J_proj.transpose() * e * weight;
            }
        }

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
            PoseOptimizer::GradientVector *g)
        {
            const Vector3d xyz_in_imu(T_imu_world * xyz_in_world);
            const Vector3d xyz_in_cam(T_cam_imu * xyz_in_imu);

            // Prediction error.
            Vector3d e = f - xyz_in_cam.normalized();
            if (unwhitened_error)
                *unwhitened_error = e.norm();

            // Whitened error.
            double R = 1.0 / measurement_sigma;
            e *= R;

            // M-estimator.
            double weight = robust_weight.Weight(e.norm());

            // Chi2 error (likelihood).
            *chi2_error = 0.5 * e.squaredNorm() * weight;

            if (H && g)
            {
                // compute jacobian
                PoseOptimizer::Matrix36d J_bearing;
                Frame::Jacobian_xyz2f_imu(T_cam_imu, xyz_in_imu, J_bearing);
                PoseOptimizer::Matrix36d J_proj = (-1.0) * J_bearing;
                J_proj *= R;
                H->noalias() += J_proj.transpose() * J_proj * weight;
                g->noalias() -= J_proj.transpose() * e * weight;
            }
        }

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
            PoseOptimizer::GradientVector *g)
        {
            const Vector3d xyz_in_imu(T_imu_world * xyz_in_world);
            const Vector3d xyz_in_cam(T_cam_imu * xyz_in_imu);

            // Compute error.
            double e = grad.dot(vk::project2(f) - vk::project2(xyz_in_cam));
            if (unwhitened_error)
                *unwhitened_error = std::abs(e);

            // Whiten error.
            double R = 1.0 / measurement_sigma;
            e *= R;

            // Robustification.
            double weight = robust_weight.Weight(e);

            // Chi2 error (likelihood).
            *chi2_error = 0.5 * e * e * weight;

            if (H && g)
            {
                // Compute Jacobian.
                PoseOptimizer::Matrix26d J_proj;
                Frame::Jacobian_xyz2uv_imu(T_cam_imu, xyz_in_imu, J_proj);
                PoseOptimizer::Vector6d J = grad.transpose() * J_proj;
                J *= R;
                H->noalias() += J * J.transpose() * weight;
                g->noalias() -= J * e * weight;
            }
        }

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
            PoseOptimizer::GradientVector *g)
        {
            const Vector3d xyz_in_imu(T_imu_world * xyz_in_world);
            const Vector3d xyz_in_cam(T_cam_imu * xyz_in_imu);

            // Prediction error.
            Eigen::Matrix<double, 2, 3> J_cam;
            Keypoint px_est;
            cam.project3(xyz_in_cam, &px_est, &J_cam);
            double e = grad.dot(px - px_est);
            if (unwhitened_error)
                *unwhitened_error = std::abs(e);

            // Whiten error: R*e, where R is the square root of information matrix (1/sigma).
            double R = 1.0 / measurement_sigma;
            e *= R;

            // M-estimator weighting.
            double weight = robust_weight.Weight(e);

            // Chi2 error, i.e. log-likelihood : 1/(2*sigma^2)*(z-h(x))^2 = 1/2*e'R'*R*e
            *chi2_error = 0.5 * e * e * weight;

            if (H && g)
            {
                // Compute Jacobian.
                PoseOptimizer::Matrix26d J_proj;
                Frame::Jacobian_xyz2img_imu(T_cam_imu, xyz_in_imu, J_cam, J_proj);
                PoseOptimizer::Vector6d J = grad.transpose() * (-1.0) * J_proj;
                J *= R;
                H->noalias() += J * J.transpose() * weight;
                g->noalias() -= J * e * weight;
            }
        }

        // NOTE: current implementation basically scales the residual on
        //       the image plane to the unit sphere. This involves the calculation
        //       of projection functions and Jacobians, which is not efficient.
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
            PoseOptimizer::GradientVector *g)
        {
            const Vector3d xyz_in_imu(T_imu_world * xyz_in_world);
            const Vector3d xyz_in_cam(T_cam_imu * xyz_in_imu);

            // Prediction error.
            Eigen::Matrix<double, 2, 3> J_cam;
            Keypoint px_est;
            cam.project3(xyz_in_cam, &px_est, &J_cam);
            Vector2d px_diff = px - px_est;
            double px_diff_norm2 = px_diff.squaredNorm();
            BearingVector f_est = xyz_in_cam.normalized();
            Vector3d f_diff = f - f_est;
            double f_diff_norm2 = f_diff.squaredNorm();
            double e_img = grad.dot(px_diff);
            double scale_ratio = f_diff.norm() / px_diff.norm();
            double e = e_img * scale_ratio;
            if (unwhitened_error)
                *unwhitened_error = std::abs(e);

            // Whiten error: R*e, where R is the square root of information matrix (1/sigma).
            double R = 1.0 / measurement_sigma;
            e *= R;

            // M-estimator weighting
            double weight = robust_weight.Weight(e);

            // Chi2 error, i.e. log-likelihood : 1/(2*sigma^2)*(z-h(x))^2 = 1/2*e'R'*R*e
            *chi2_error = 0.5 * e * e * weight;

            if (H && g)
            {
                // Compute Jacobian.
                PoseOptimizer::Matrix26d J_proj;
                Frame::Jacobian_xyz2img_imu(T_cam_imu, xyz_in_imu, J_cam, J_proj);
                PoseOptimizer::Matrix36d J_bearing;
                Frame::Jacobian_xyz2f_imu(T_cam_imu, xyz_in_imu, J_bearing);

                PoseOptimizer::Vector6d J_img = grad.transpose() * (-1.0) * J_proj;

                PoseOptimizer::Vector6d J_ftf = 2 * f_diff.transpose() * (-1.0) * J_bearing;
                PoseOptimizer::Vector6d J_ptp = 2 * px_diff.transpose() * (-1.0) * J_proj;
                PoseOptimizer::Vector6d J_ratio =
                    (0.5) * (1.0 / (scale_ratio)) * (1 / (px_diff_norm2 * px_diff_norm2)) *
                    (J_ftf * px_diff_norm2 - J_ptp * f_diff_norm2);

                PoseOptimizer::Vector6d J = e_img * J_ratio + scale_ratio * J_img;

                J *= R;
                H->noalias() += J * J.transpose() * weight;
                g->noalias() -= J * e * weight;
            }
        }

    } // namespace pose_optimizer_utils
} // namespace mivins
