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
#include <mivins/transformation_optimizer.h>
#include <mivins/common/seed.h>

namespace mivins
{

    TransformationOptimizer::TransformationOptimizer(SolverOptions solver_options)
        : mivins::MiniLeastSquaresSolver<6, Transformation, TransformationOptimizer>(solver_options)
    {
    }

    TransformationOptimizer::SolverOptions TransformationOptimizer::getDefaultSolverOptions()
    {
        SolverOptions options;
        options.strategy = mivins::Strategy::GaussNewton;
        options.max_iter = 10;
        options.eps = 0.000001;
        return options;
    }

    void TransformationOptimizer::SetTranslationPrior(const Eigen::Vector3d &t_prior, double lambda)
    {
        Quaternion R;
        R.SetIdentity();
        Transformation T_prior(R, t_prior);
        Matrix6d Information = Matrix6d::Zero();
        Information.topLeftCorner<3,3>() = Eigen::Matrix3d::Identity();
        prior_lambda_ = lambda;
        SetPrior(T_prior, Information);
    }

    void TransformationOptimizer::run(PoseDeque &vio_pose, PoseDeque &tar_pose, Transformation &T, const double outlier_th, double &inlier_error, double &inlier_ratio)
    {
        CHECK(!vio_pose.empty() || !tar_pose.empty()) << "Transformation: Pose deque is empty.";
        vio_pose_ = vio_pose;
        tar_pose_ = tar_pose;
        pose_num_ = vio_pose.size();

        std::vector<float> start_errors;
        EvaluateErrorImpl(T, nullptr, nullptr, &start_errors);
        measurement_sigma_ = sigma_computer_.Compute(start_errors);
        VLOG(5) << "Initial measurement sigma:" << measurement_sigma_;

        optimize(T);

        std::vector<double> final_errors;
        CheckOutliers(outlier_th, T, vio_pose, tar_pose, &final_errors, &inlier_error, &inlier_ratio);

        return;
    }

    double TransformationOptimizer::EvaluateError(
        const Transformation &T,
        HessianMatrix *H,
        GradientVector *g)
    {
        return EvaluateErrorImpl(T, H, g, nullptr);
    }

    double TransformationOptimizer::EvaluateErrorImpl(
        const Transformation &T,
        HessianMatrix *H,
        GradientVector *g,
        std::vector<float> *unwhitened_errors)
    {
        double chi2_error_sum = 0.0;

        // compute the weights on the first iteration
        if (unwhitened_errors)
            unwhitened_errors->reserve(pose_num_);

        for (int i=0; i<pose_num_; ++i)
        {
            Transformation T_vio = vio_pose_[i];
            Transformation T_tar = tar_pose_[i];

            double unwhitened_error, chi2_error;
            transformation_optimizer_utils::CalculateResidualHessianJacobian(
                T, T_vio, T_tar, measurement_sigma_, loss_function_, &unwhitened_error,
                &chi2_error, H, g);

            if (unwhitened_errors)
            {
                CHECK_GE(unwhitened_error, 0.0);
                unwhitened_errors->push_back(unwhitened_error);
            }
            chi2_error_sum += chi2_error;
            ++m_meas_size;
        }

        return chi2_error_sum;
    }

    void TransformationOptimizer::Update(
        const State &T_old,
        const UpdateVector &dx,
        State &T_new)
    {
        T_new = Transformation::Exp(dx) * T_old;

        // we need to normalize from time to time, otherwise rounding errors sum up
        T_new.GetRotation().ToImplementation().normalize();
    }

    void TransformationOptimizer::CheckOutliers(
        const double outlier_th,
        const Transformation &T,
        PoseDeque &vio_pose,
        PoseDeque &tar_pose,
        std::vector<double> *unwhitened_errors,
        double *inlier_error,
        double *inlier_ratio)
    {
        unwhitened_errors->reserve(vio_pose.size());
        *inlier_error = 0.0;
        *inlier_ratio = 1.0;
        int inlier_num = 0;

        for(int i=0; i<vio_pose.size(); ++i)
        {
            Transformation T_vio = vio_pose[i];
            Transformation T_tar = tar_pose[i];

            Matrix<double, 6, 1> e = Matrix<double, 6, 1>::Zero();
            e.head(3) = T_tar.GetPosition() - (T.GetRotation().Rotate(T_vio.GetPosition())+T.GetPosition());
            e.tail(3) = (T_tar.GetRotation().Inverse()*T.GetRotation()*T_vio.GetRotation()).Log();
            double unwhitened_error = e.norm();

            if(unwhitened_errors)
            {
                CHECK_GE(unwhitened_error, 0.0);
                unwhitened_errors->push_back(unwhitened_error);
            }

            // trace_optimizer_ << unwhitened_error << std::endl;

            if(unwhitened_error > outlier_th)
                continue;

            *inlier_error += unwhitened_error;
            ++inlier_num;
        }

        *inlier_error /= static_cast<double>(inlier_num);
        *inlier_ratio  = static_cast<double>(inlier_num)/static_cast<double>(vio_pose.size());

        return;
    }

    void TransformationOptimizer::ApplyPrior(const State &T_prior)
    {
        if (m_iter == 0)
        {
            m_info_prior = Matrix6d::Zero();
            m_info_prior.topLeftCorner<3, 3>() = Matrix3d::Identity();

            double H_max_diag = 0;
            //double tau = 1e-4;
            for (size_t j = 0; j < 3; ++j)
                H_max_diag = std::max(H_max_diag, std::fabs(m_H(j, j)));
            m_info_prior *= H_max_diag * prior_lambda_;
        
            if (m_solver_options.verbose)
            {
                std::cout << "applying translation prior, I = " << H_max_diag * prior_lambda_ << std::endl;
            }
        }

        m_H.noalias() += m_info_prior;
        m_g.noalias() -= m_info_prior * Transformation::Log(T_prior * m_state_prior.Inverse());
        //std::cout << "information matrix = " << m_info_prior << std::endl;
    }

    namespace transformation_optimizer_utils
    {
        Eigen::Matrix3d computeRightJacobian(Quaternion &R)
        {
            Eigen::Matrix3d Jr;

            Eigen::AngleAxisd aa(R.GetRotationMatrix());
            Eigen::Vector3d ax = aa.axis();
            double theta = aa.angle();

            Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
            Jr = (sin(theta)/theta)*I3 + (1-sin(theta)/theta)*ax*ax.transpose() - ((1-cos(theta))/theta)*vk::skew(ax);

            if(Jr.hasNaN())
                Jr.setZero();

            return Jr;
        }

        Eigen::Matrix3d computeRightJacobianInverse(Quaternion &R)
        {
            Eigen::Matrix3d Jri;

            Eigen::AngleAxisd aa(R.GetRotationMatrix());
            Eigen::Vector3d ax = aa.axis();
            double theta = aa.angle();

            theta /= 2.0;
            Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
            Jri = theta*cot(theta)*I3 + (1-theta*cot(theta))*ax*ax.transpose() + theta*vk::skew(ax);

            if(Jri.hasNaN())
                Jri.setZero();

            return Jri;
        }

        double cot(double &x)
        {
            return tan(M_PI/2.0 - x);
        }

        void CalculateResidualHessianJacobian(
            const Transformation &T,
            const Transformation &T_vio,
            const Transformation &T_tar,
            double measurement_sigma,
            const TransformationOptimizer::LossFunction &robust_weight,
            double *unwhitened_error,
            double *chi2_error,
            TransformationOptimizer::HessianMatrix *H,
            TransformationOptimizer::GradientVector *g)
        {
            // Prediction error.
            Matrix<double, 6, 1> e = Matrix<double, 6, 1>::Zero();
            e.head(3) = T_tar.GetPosition() - (T.GetRotation().Rotate(T_vio.GetPosition())+T.GetPosition());
            e.tail(3) = (T_tar.GetRotation().Inverse()*T.GetRotation()*T_vio.GetRotation()).Log();

            if (unwhitened_error)
                *unwhitened_error = e.norm();

            // Whiten error: R*e, where R is the square root of information matrix (1/sigma).
            double R = 1.0 / measurement_sigma;
            // e *= R;
            Eigen::Matrix<double,6,6> R_d = Eigen::Matrix<double,6,6>::Identity();
            R_d.block<3,3>(0,0) = 3.0*R*Eigen::Matrix3d::Identity();
            R_d.block<3,3>(3,3) = 1.0*R*Eigen::Matrix3d::Identity();
            e = R_d*e;

            // M-estimator weighting
            double weight = robust_weight.Weight(e.norm());

            // Compute log-likelihood : 1/(2*sigma^2)*(z-h(x))^2 = 1/2*e'R'*R*e
            *chi2_error = 0.5 * e.squaredNorm() * weight;

            if (H && g)
            {
                // compute jacobian
                TransformationOptimizer::Matrix6d J;

                Quaternion R_tmp1 = T.GetRotation()*T_vio.GetRotation();
                Quaternion R_tmp2 = T_tar.GetRotation().Inverse()*R_tmp1;

                J.block<3,3>(0,0) = -1.0 * Eigen::Matrix3d::Identity();
                J.block<3,3>(3,0) = Eigen::Matrix3d::Zero();
                J.block<3,3>(0,3) = vk::skew(T.GetRotation().Rotate(T_vio.GetPosition()));
                J.block<3,3>(3,3) = transformation_optimizer_utils::computeRightJacobianInverse(R_tmp2)*R_tmp1.GetRotationMatrix().transpose();

                // J *= R;
                J = R_d*J;
                H->noalias() += J.transpose() * J * weight;
                g->noalias() -= J.transpose() * e * weight;
            }
        }
    }
} // namespace mivins
