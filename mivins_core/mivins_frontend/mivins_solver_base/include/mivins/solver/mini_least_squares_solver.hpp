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

#include "mivins/solver/mini_least_squares_solver.h"

#include <stdexcept>
#include <glog/logging.h>

namespace mivins
{
    namespace utils
    {

        inline double Norm_max(const Eigen::VectorXd &v)
        {
            double max = -1;
            for (int i = 0; i < v.size(); i++)
            {
                double abs = std::fabs(v[i]);
                if (abs > max)
                {
                    max = abs;
                }
            }
            return max;
        }

    } // namespace utils

    template <int D, typename T, typename Implementation>
    MiniLeastSquaresSolver<D, T, Implementation>::MiniLeastSquaresSolver(
        const MiniLeastSquaresSolverOptions &options)
        : m_solver_options(options)
    {
    }

    template <int D, typename T, typename Implementation>
    void MiniLeastSquaresSolver<D, T, Implementation>::optimize(State &state)
    {
        if (m_solver_options.strategy == Strategy::GaussNewton)
            OptimizeGaussNewton(state);
        else if (m_solver_options.strategy == Strategy::LevenbergMarquardt)
            OptimizeLevenbergMarquardt(state);
    }

    template <int D, typename T, typename Implementation>
    void MiniLeastSquaresSolver<D, T, Implementation>::OptimizeGaussNewton(State &state)
    {
        // Save the old model to rollback in case of unsuccessful update
        State old_state = state;

        // perform iterative estimation
        for (m_iter = 0; m_iter < m_solver_options.max_iter; ++m_iter)
        {
            m_rho = 0;
            StartIteration();

            m_H.setZero();
            m_g.setZero();

            // compute initial error
            m_meas_size = 0;
            double new_chi2 = EvaluateError(state, &m_H, &m_g);

            // add prior
            if (m_have_prior)
            {
                ApplyPrior(state);
            }

            // solve the linear system
            if (!Solve(m_H, m_g, m_dx))
            {
                LOG(WARNING) << "Matrix is close to singular! Stop Optimizing."
                             << "H = " << m_H << "g = " << m_g;
                m_stop = true;
            }

            // check if error increased since last optimization
            if ((m_iter > 0 && new_chi2 > m_chi2 && m_solver_options.stop_when_error_increases) || m_stop)
            {
                VLOG(400) << "It. " << m_iter
                          << "\t Failure"
                          << "\t new_chi2 = " << new_chi2
                          << "\t n_meas = " << m_meas_size
                          << "\t Error increased. Stop optimizing.";
                state = old_state; // rollback
                break;
            }

            // update the model
            State new_state;
            Update(state, m_dx, new_state);
            old_state = state;
            state = new_state;
            m_chi2 = new_chi2;
            //double x_norm = utils::norm_max(m_dx);
            double x_norm = utils::Norm_max(m_dx);
            VLOG(400) << "It. " << m_iter
                      << "\t Success"
                      << "\t new_chi2 = " << new_chi2
                      << "\t n_meas = " << m_meas_size
                      << "\t x_norm = " << x_norm;
            FinishIteration();

            // stop when converged, i.e. update step too small
            if (x_norm < m_solver_options.eps)
            {
                VLOG(400) << "Converged, x_norm " << x_norm << " < " << m_solver_options.eps;
                break;
            }
        }
    }

    template <int D, typename T, typename Implementation>
    void MiniLeastSquaresSolver<D, T, Implementation>::OptimizeLevenbergMarquardt(State &state)
    {
        // init parameters
        m_mu = m_solver_options.mu_init;
        m_nu = m_solver_options.nu_init;

        // compute the initial error
        m_chi2 = EvaluateError(state, nullptr, nullptr);
        VLOG(400) << "init chi2 = " << m_chi2
                  << "\t n_meas = " << m_meas_size;

        // TODO: compute initial lambda
        // Hartley and Zisserman: "A typical init value of lambda is 10^-3 times the
        // average of the diagonal elements of J'J"
        // Compute Initial Lambda
        if (m_mu < 0)
        {
            double H_max_diag = 0;
            double tau = 1e-4;
            for (size_t j = 0; j < D; ++j)
            {
                H_max_diag = std::max(H_max_diag, std::fabs(m_H(j, j)));
            }
            m_mu = tau * H_max_diag;
        }

        // perform iterative estimation
        for (m_iter = 0; m_iter < m_solver_options.max_iter; ++m_iter)
        {
            m_rho = 0;
            StartIteration();

            // try to compute and update, if it fails, try with increased mu
            m_trials = 0;
            do
            {
                // init variables
                State new_model;
                double new_chi2 = -1;
                m_H.setZero();
                //m_H = m_mu * Matrix<double,D,D>::Identity(D,D);
                m_g.setZero();

                // linearize
                m_meas_size = 0;
                EvaluateError(state, &m_H, &m_g);

                // add damping term:
                m_H += (m_H.diagonal() * m_mu).asDiagonal();

                // add prior
                if (m_have_prior)
                {
                    ApplyPrior(state);
                }

                // solve the linear system to obtain small perturbation in direction of gradient
                if (Solve(m_H, m_g, m_dx))
                {
                    // apply perturbation to the state
                    Update(state, m_dx, new_model);

                    // compute error with new model and compare to old error
                    m_meas_size = 0;
                    new_chi2 = EvaluateError(new_model, nullptr, nullptr);
                    m_rho = m_chi2 - new_chi2;
                }
                else
                {
                    LOG(WARNING) << "Matrix is close to singular! Stop Optimizing."
                                 << "H = " << m_H << "g = " << m_g;
                    m_rho = -1;
                }

                if (m_rho > 0)
                {
                    // update decrased the error -> success
                    state = new_model;
                    m_chi2 = new_chi2;
                    //m_stop = utils::norm_max(m_dx) < m_solver_options.eps;
                    m_stop = utils::Norm_max(m_dx) < m_solver_options.eps;
                    m_mu *= std::max(1. / 3., std::min(1. - std::pow(2 * m_rho - 1, 3), 2. / 3.));
                    m_nu = 2.;
                    VLOG(400) << "It. " << m_iter
                              << "\t Trial " << m_trials
                              << "\t Success"
                              << "\t n_meas = " << m_meas_size
                              << "\t new_chi2 = " << new_chi2
                              << "\t mu = " << m_mu
                              << "\t nu = " << m_nu;
                }
                else
                {
                    // update increased the error -> fail
                    m_mu *= m_nu;
                    m_nu *= 2.;
                    ++m_trials;
                    if (m_trials >= m_solver_options.max_trials)
                        m_stop = true;

                    VLOG(400) << "It. " << m_iter
                              << "\t Trial " << m_trials
                              << "\t Failure"
                              << "\t n_meas = " << m_meas_size
                              << "\t new_chi2 = " << new_chi2
                              << "\t mu = " << m_mu
                              << "\t nu = " << m_nu;
                }
                FinishTrial();

            } while (!(m_rho > 0 || m_stop));
            if (m_stop)
            {
                break;
            }

            FinishIteration();
        }
    }

    template <int D, typename T, typename Implementation>
    void MiniLeastSquaresSolver<D, T, Implementation>::SetPrior(
        const T &prior,
        const Matrix<double, D, D> &Information)
    {
        m_have_prior = true;
        m_state_prior = prior;
        m_info_prior = Information;
    }

    template <int D, typename T, typename Implementation>
    void MiniLeastSquaresSolver<D, T, Implementation>::Reset()
    {
        m_have_prior = false;
        m_chi2 = 1e10;
        m_mu = m_solver_options.mu_init;
        m_nu = m_solver_options.nu_init;
        m_meas_size = 0;
        m_iter = 0;
        m_trials = 0;
        m_stop = false;
    }

    template <int D, typename T, typename Implementation>
    bool MiniLeastSquaresSolver<D, T, Implementation>::SolveDefaultImpl(
        const HessianMatrix &H,
        const GradientVector &g,
        UpdateVector &dx)
    {
        dx = H.ldlt().solve(g);
        if ((bool)std::isnan((double)dx[0]))
            return false;
        return true;
    }
} // namespace mivins
