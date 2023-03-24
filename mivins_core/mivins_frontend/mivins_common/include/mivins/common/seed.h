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

#pragma once

#include <glog/logging.h>
#include <mivins/common/types.h>

namespace mivins
{
    namespace seed
    {

        enum SeedStateIndex
        {
            kMu,
            kSigma2,
            kA,
            kB,
        };

        // -----------------------------------------------------------------------------
        // Accessors

        inline FloatType mu(const Eigen::Ref<const SeedState> &mu_sigma2_a_b)
        {
            return mu_sigma2_a_b(0);
        }

        inline FloatType sigma2(const Eigen::Ref<const SeedState> &mu_sigma2_a_b)
        {
            return mu_sigma2_a_b(1);
        }

        inline FloatType a(const Eigen::Ref<const SeedState> &mu_sigma2_a_b)
        {
            return mu_sigma2_a_b(2);
        }

        inline FloatType b(const Eigen::Ref<const SeedState> &mu_sigma2_a_b)
        {
            return mu_sigma2_a_b(3);
        }

#if 0
// -----------------------------------------------------------------------------
// Exponential Depth Parametrization

inline FloatType getDepth(const Eigen::Ref<const SeedState>& mu_sigma2_a_b)
{
  return std::exp(mu_sigma2_a_b(0));
}

inline FloatType getInvDepth(const Eigen::Ref<const SeedState>& mu_sigma2_a_b)
{
  return std::exp(-mu_sigma2_a_b(0)); // exp(x)^-1 = exp(-x)
}

inline FloatType getInvMinDepth(const Eigen::Ref<const SeedState>& mu_sigma2_a_b)
{
  const FloatType sigma = std::sqrt(mu_sigma2_a_b(1));
  return std::exp(-(mu_sigma2_a_b(0) - sigma));
}

inline FloatType getInvMaxDepth(const Eigen::Ref<const SeedState>& mu_sigma2_a_b)
{
  const FloatType sigma = std::sqrt(mu_sigma2_a_b(1));
  return std::exp(-(mu_sigma2_a_b(0) + sigma));
}

inline FloatType getMeanFromDepth(FloatType depth)
{
  return std::log(depth);
}

inline FloatType getMeanRangeFromDepthMinMax(FloatType depth_min, FloatType depth_max)
{
  CHECK_GE(depth_max, 0.0);
  CHECK_GE(depth_min, 0.0);
  return (std::log(depth_max) - std::log(depth_min))*2.0;
}

inline FloatType getInitSigma2FromMuRange(FloatType mu_range)
{
  return mu_range * mu_range / 36.0;
}

inline bool isConverged(const Eigen::Ref<const SeedState>& mu_sigma2_a_b,
                        FloatType mu_range,
                        FloatType sigma2_convergence_threshold)
{
  // If initial uncertainty was reduced by factor sigma2_convergence_threshold
  // we accept the seed as converged.
  const FloatType thresh = mu_range / sigma2_convergence_threshold;
  return (mu_sigma2_a_b(1) < thresh * thresh);
}

inline FloatType getSigma2FromDepthSigma(FloatType depth, FloatType depth_sigma)
{
  const FloatType sigma = 0.5 * (std::log(depth+depth_sigma) - std::log(depth-depth_sigma));
  return sigma * sigma;
}
#else
        // -----------------------------------------------------------------------------
        // Inverse Depth Parametrization

        inline FloatType getDepth(const Eigen::Ref<const SeedState> &mu_sigma2_a_b)
        {
            return 1.0 / mu_sigma2_a_b(0);
        }

        inline FloatType getInvDepth(const Eigen::Ref<const SeedState> &mu_sigma2_a_b)
        {
            return mu_sigma2_a_b(0);
        }

        inline FloatType getInvMinDepth(const Eigen::Ref<const SeedState> &mu_sigma2_a_b)
        {
            return mu_sigma2_a_b(0) + std::sqrt(mu_sigma2_a_b(1));
        }

        inline FloatType getInvMaxDepth(const Eigen::Ref<const SeedState> &mu_sigma2_a_b)
        {
            return std::max(mu_sigma2_a_b(0) - std::sqrt(mu_sigma2_a_b(1)), 0.00000001);
        }

        inline FloatType getMeanFromDepth(FloatType depth)
        {
            return 1.0 / depth;
        }

        inline FloatType getMeanRangeFromDepthMinMax(FloatType depth_min, FloatType /*depth_max*/)
        {
            return 1.0 / depth_min;
        }

        inline FloatType getInitSigma2FromMuRange(FloatType mu_range)
        {
            return mu_range * mu_range / 36.0;
        }

        inline bool isConverged(const Eigen::Ref<const SeedState> &mu_sigma2_a_b,
                                FloatType mu_range,
                                FloatType sigma2_convergence_threshold)
        {
            // If initial uncertainty was reduced by factor sigma2_convergence_threshold
            // we accept the seed as converged.
            const FloatType thresh = mu_range / sigma2_convergence_threshold;
            return (mu_sigma2_a_b(1) < thresh * thresh);
        }

        inline FloatType getSigma2FromDepthSigma(FloatType depth, FloatType depth_sigma)
        {
            const FloatType sigma = 0.5 * (1.0 / std::max(0.000000000001, depth - depth_sigma) - 1.0 / (depth + depth_sigma));
            return sigma * sigma;
        }
#endif

        // -----------------------------------------------------------------------------
        // Utils

        inline void increaseOutlierProbability(Eigen::Ref<SeedState> &mu_sigma2_a_b)
        {
            mu_sigma2_a_b(3) += 1;
        }

    } // namespace seed
} // namespace mivins
