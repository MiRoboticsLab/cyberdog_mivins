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

#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/StdVector>

namespace mivins
{
    using namespace Eigen;

    enum class Strategy
    {
        GaussNewton,
        LevenbergMarquardt
    };

    struct MiniLeastSquaresSolverOptions
    {
        /// Solver strategy.
        Strategy strategy = Strategy::GaussNewton;

        /// Damping parameter. If mu > 0, coefficient matrix is positive definite, this
        /// ensures that x is a descent direction. If mu is large, x is a short step in
        /// the steepest direction. This is good if the current iterate is far from the
        /// solution. If mu is small, LM approximates gauss newton iteration and we
        /// have (almost) quadratic convergence in the final stages.
        double mu_init = 0.01f;

        /// Increase factor of mu after fail
        double nu_init = 2.0;

        /// Max number of iterations
        size_t max_iter = 15;

        /// Max number of trials (used in LevenbergMarquardt)
        size_t max_trials = 5;

        /// Stop when error increases.
        bool stop_when_error_increases = false;

        /// Output Statistics
        bool verbose = false;

        /// Stop if update norm is smaller than eps
        double eps = 0.0000000001;
    };

    /// Abstract Class for solving nonlinear least-squares (NLLS) problems.
    /// Template Parameters: D  : dimension of the residual, T: type of the model
    /// e.g. SE2, SE3
    template <int D, typename T, typename Implementation>
    class MiniLeastSquaresSolver
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef T State;
        typedef Matrix<double, D, D> HessianMatrix;
        typedef Matrix<double, D, 1> GradientVector;
        typedef Matrix<double, D, 1> UpdateVector;

        MiniLeastSquaresSolverOptions m_solver_options;

    protected:
        MiniLeastSquaresSolver() = default;

        MiniLeastSquaresSolver(const MiniLeastSquaresSolverOptions &options);

        virtual ~MiniLeastSquaresSolver() = default;

    public:
        /// Calls the GaussNewton or LevenbergMarquardt optimization strategy
        void optimize(State &state);

        /// Gauss Newton optimization strategy
        void OptimizeGaussNewton(State &state);

        /// Levenberg Marquardt optimization strategy
        void OptimizeLevenbergMarquardt(State &state);

        /// Add prior to optimization.
        void SetPrior(
            const State &prior,
            const Matrix<double, D, D> &Information);

        /// Reset all parameters to restart the optimization
        void Reset();

        /// Get the squared error
        inline double GetError() const
        {
            return m_chi2;
        }

        /// The the Hessian matrix (Information Matrix).
        inline const Matrix<double, D, D> &GetHessian() const
        {
            return m_H;
        }

    protected:
        Implementation &Impl()
        {
            return *static_cast<Implementation *>(this);
        }

        /// Evaluates the error at provided state. Optional return variables are
        /// the Hessian matrix and the gradient vector (Jacobian * residual).
        /// If these parameters are requested, the system is linearized at the current
        /// state.
        double EvaluateError(
            const State &state,
            HessianMatrix *H,
            GradientVector *g)
        {
            return Impl().EvaluateError(state, H, g);
        }

        /// Solve the linear system H*dx = g to obtain optimal perturbation dx.
        bool Solve(
            const HessianMatrix &H,
            const GradientVector &g,
            UpdateVector &dx)
        {
            if (&MiniLeastSquaresSolver::Solve != &Implementation::Solve)
                return Impl().Solve(H, g, dx);
            else
                return SolveDefaultImpl(H, g, dx);
        }

        /// Apply the perturbation dx to the state.
        void Update(
            const State &state,
            const UpdateVector &dx,
            State &new_state)
        {
            Impl().Update(state, dx, new_state);
        }

        void ApplyPrior(const State &current_model)
        {
            if (&MiniLeastSquaresSolver::ApplyPrior != &Implementation::ApplyPrior)
                Impl().ApplyPrior(current_model);
        }

        void StartIteration()
        {
            if (&MiniLeastSquaresSolver::StartIteration != &Implementation::StartIteration)
                Impl().StartIteration();
        }

        void FinishIteration()
        {
            if (&MiniLeastSquaresSolver::FinishIteration != &Implementation::FinishIteration)
                Impl().FinishIteration();
        }

        void FinishTrial()
        {
            if (&MiniLeastSquaresSolver::FinishTrial != &Implementation::FinishTrial)
                Impl().FinishTrial();
        }

    private:
        /// Default implementation to solve the linear system H*dx = g to obtain optimal perturbation dx.
        bool SolveDefaultImpl(
            const HessianMatrix &H,
            const GradientVector &g,
            UpdateVector &dx);

    protected:
        HessianMatrix m_H;  ///< Hessian or approximation Jacobian*Jacobian^T.
        GradientVector m_g; ///< Jacobian*residual.
        UpdateVector m_dx;  ///< Update step.
        bool m_have_prior = false;
        State m_state_prior;
        Matrix<double, D, D> m_info_prior; ///< Prior information matrix (inverse covariance)
        double m_chi2 = 0.0;               ///< Whitened error / log-likelihood: 1/(2*sigma^2)*(z-h(x))^2.
        double m_rho = 0.0;                ///< Error reduction: chi2-new_chi2.
        double m_mu = 0.01;                ///< Damping parameter.
        double m_nu = 2.0;                 ///< Factor that specifies how much we increase mu at every trial.
        size_t m_meas_size = 0;            ///< Number of measurements.
        bool m_stop = false;               ///< Stop flag.
        size_t m_iter = 0;                 ///< Current Iteration.
        size_t m_trials = 0;               ///< Current number of trials.
    };
} // namespace mivins

#include "mivins/solver/mini_least_squares_solver.hpp"
