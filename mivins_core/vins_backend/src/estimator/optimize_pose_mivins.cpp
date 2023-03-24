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

#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "estimator/optimize_pose_mivins.h"

OptimizePose::OptimizePose(SolverOptions solver_options)
    : mivins::MiniLeastSquaresSolver<6, mivins::Transformation, OptimizePose>(solver_options)
{}

OptimizePose::SolverOptions OptimizePose::getDefaultSolverOptions()
{
    SolverOptions options;
    options.strategy = mivins::Strategy::GaussNewton;
    options.max_iter = 10;
    options.eps = 0.000001;
    return options;
}

void OptimizePose::setRotationPrior(const mivins::Quaternion& R_frame_world, double lambda)
{
    mivins::Transformation T_cur_world_prior(R_frame_world, Eigen::Vector3d::Zero());
    Matrix6d Information = Matrix6d::Zero();
    Information.bottomRightCorner<3,3>() = Eigen::Matrix3d::Identity();
    prior_lambda_ = lambda;
    SetPrior(T_cur_world_prior, Information);
}

void OptimizePose::setTransformPrior(const mivins::Transformation &T_cur_world_prior, double lambda)
{
    Matrix6d Information = Matrix6d::Zero();
    Information.bottomRightCorner<3,3>() = Eigen::Matrix3d::Identity();
    prior_lambda_ = lambda;
    SetPrior(T_cur_world_prior, Information);
}

void OptimizePose::run(const mivins::FrameBundle::Ptr& frame_bundle, 
    const std::vector<std::unordered_map<size_t, Eigen::Vector3d>> &id_pts_maps)
{
    id_pts_maps_ = id_pts_maps;
    frame_bundle_ = frame_bundle;
    focal_length_ = frame_bundle->at(0)->GetErrorMultiplier();
    
    mivins::Transformation T_imu_world = frame_bundle->at(0)->T_imu_world();

    // Check the scale of the errors.
    std::vector<float> start_errors;
    evaluateErrorImpl(T_imu_world, nullptr, nullptr, &start_errors);
    measurement_sigma_ = sigma_computer_.Compute(start_errors);
    VLOG(5) << "Initial measurement sigma:" << measurement_sigma_;

    optimize(T_imu_world);

    for(const mivins::FramePtr& frame : frame_bundle->frames_)
    {
        frame->T_f_w_ = frame->T_cam_imu() * T_imu_world;
    }
}

double OptimizePose::EvaluateError(
        const mivins::Transformation& T_imu_world,
        HessianMatrix* H,
        GradientVector* g)
{
    return evaluateErrorImpl(T_imu_world, H, g, nullptr);
}

double OptimizePose::evaluateErrorImpl(
        const mivins::Transformation& T_imu_world,
        HessianMatrix* H,
        GradientVector* g,
        std::vector<float>* unwhitened_errors)
{
    double chi2_error_sum = 0.0;

    // compute the weights on the first iteration
    if(unwhitened_errors)
        unwhitened_errors->reserve(frame_bundle_->NumFeatures());

    int frame_idx = 0;
    for(const mivins::FramePtr& frame : frame_bundle_->frames_)
    {
        const mivins::Transformation T_cam_imu = frame->T_cam_imu();
        auto &id_pts_map = id_pts_maps_[frame_idx];

        for(size_t i = 0; i < frame->num_features_; ++i)
        {
            size_t feature_id = frame->track_id_vec_(i);
            if(id_pts_map.find(feature_id) == id_pts_map.end())
                continue;

            Eigen::Vector3d xyz_world = id_pts_map[feature_id];
            
            const int scale = (1 << frame->level_vec_(i));
            double unwhitened_error, chi2_error;
            double measurement_sigma = measurement_sigma_ * scale;
            if(isEdgelet(frame->type_vec_[i]))
            {
                // Edgelets should have less weight than corners.
                constexpr double kEdgeletSigmaExtraFactor = 2.0;
                measurement_sigma *= kEdgeletSigmaExtraFactor;
                if(err_type_ == ErrorType::kUnitPlane)
                    mivins::pose_optimizer_utils::CalculateEdgeletResidualUnitPlane(
                                frame->f_vec_.col(i), xyz_world, frame->grad_vec_.col(i),
                                T_imu_world, T_cam_imu, measurement_sigma, loss_function_,
                                &unwhitened_error, &chi2_error, H, g);
                else if(err_type_ == ErrorType::kImagePlane)
                    mivins::pose_optimizer_utils::CalculateEdgeletResidualImagePlane(
                                frame->px_vec_.col(i), xyz_world, frame->grad_vec_.col(i),
                                T_imu_world, T_cam_imu, *frame->cam(), measurement_sigma,
                                loss_function_, &unwhitened_error, &chi2_error, H, g);
                else if(err_type_ == ErrorType::kBearingVectorDiff)
                    mivins::pose_optimizer_utils::CalculateEdgeletResidualBearingVectorDiff(
                                frame->px_vec_.col(i), frame->f_vec_.col(i), xyz_world, frame->grad_vec_.col(i),
                                T_imu_world, T_cam_imu, *frame->cam(), measurement_sigma,
                                loss_function_, &unwhitened_error, &chi2_error, H, g);
            }
            else
            {
                if(err_type_ == ErrorType::kUnitPlane)
                    mivins::pose_optimizer_utils::CalculateFeatureResidualUnitPlane(
                                frame->f_vec_.col(i), xyz_world, T_imu_world, T_cam_imu,
                                measurement_sigma, loss_function_,
                                &unwhitened_error, &chi2_error, H, g);
                else if(err_type_ == ErrorType::kImagePlane)
                {
                    mivins::pose_optimizer_utils::CalculateFeatureResidualImagePlane(
                                frame->px_vec_.col(i), xyz_world, T_imu_world, T_cam_imu,
                                *frame->cam(), measurement_sigma, loss_function_,
                                &unwhitened_error, &chi2_error, H, g);
                }
                else if(err_type_ == ErrorType::kBearingVectorDiff)
                {
                    mivins::pose_optimizer_utils::CalculateFeatureResidualBearingVectorDiff(
                                frame->f_vec_.col(i), xyz_world, T_imu_world, T_cam_imu,
                                measurement_sigma, loss_function_,
                                &unwhitened_error, &chi2_error, H, g);
                }
            }
            if(unwhitened_errors)
            {
                CHECK_GE(unwhitened_error, 0.0);
                unwhitened_errors->push_back(unwhitened_error / scale);
            }
            chi2_error_sum += chi2_error;
            ++m_meas_size;
        } // for each feature
        ++frame_idx;
    } // for each frame

    return chi2_error_sum;
}

void OptimizePose::Update(
        const State& T_imuold_world,
        const UpdateVector& dx,
        State& T_imunew_world)
{
    T_imunew_world = mivins::Transformation::Exp(dx)*T_imuold_world;

    // we need to normalize from time to time, otherwise rounding errors sum up
    T_imunew_world.GetRotation().ToImplementation().normalize();
}

void OptimizePose::ApplyPrior(const State& T_cur_from_world)
{
    if(m_iter == 0)
    {
         m_info_prior = Matrix6d::Zero();
         m_info_prior.bottomRightCorner<3,3>() = Eigen::Matrix3d::Identity();

        double H_max_diag = 0;
        //double tau = 1e-4;
        for(size_t j=3; j<6; ++j)
            H_max_diag = std::max(H_max_diag, std::fabs(m_H(j,j)));
        m_info_prior *= H_max_diag*prior_lambda_;
        //std::cout << "prior_lambda (pose_opt): " << prior_lambda_ << std::endl;
        //std::cout << H_max_diag << std::endl;
        if(m_solver_options.verbose)
        {
            std::cout << "applying rotation prior, I = " << H_max_diag*prior_lambda_ << std::endl;
        }
    }

    m_H.noalias() += m_info_prior;
    //Jres_.noalias() += m_info_prior*Transformation::log(m_state_prior*T_cur_from_world.inverse());
    m_g.noalias() -= m_info_prior*mivins::Transformation::Log(T_cur_from_world*m_state_prior.Inverse());
    //std::cout << "information matrix = " << m_info_prior << std::endl;
}

