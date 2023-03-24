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

#include "mivins/solver/loss_function.h"

#include <cmath>
#include <numeric>
#include <algorithm>
#include <glog/logging.h>

namespace mivins
{
    /* ************************************************************************* */
    // Scale Estimators
    /* ************************************************************************* */

    float MedianSigmaComputer::Compute(std::vector<float> &errors) const
    {
        CHECK(!errors.empty()) << "Error vector is empty.";
        auto it = errors.begin() + std::floor(errors.size() / 2);
        std::nth_element(errors.begin(), it, errors.end()); // compute median
        return 1.48f * (*it);                               // 1.48f / 0.6745
    }

    float NormalDistributionSigmaComputer::Compute(std::vector<float> &errors) const
    {
        const float mean = std::accumulate(errors.begin(), errors.end(), 0) / errors.size();
        float var = 0.0;
        for (const float d : errors)
            var += (d - mean) * (d - mean);
        return std::sqrt(var); // return standard deviation
    }

    /* ************************************************************************* */
    // LOSS Functions
    /* ************************************************************************* */

    TukeyLossFunction::TukeyLossFunction(const float b)
        : m_b_square_(b * b)
    {
    }

    float TukeyLossFunction::Weight(const float &error) const
    {
        const float x_square = error * error;
        if (x_square <= m_b_square_)
        {
            const float tmp = 1.0f - x_square / m_b_square_;
            return tmp * tmp;
        }
        else
        {
            return 0.0f;
        }
    }

    HuberLossFunction::HuberLossFunction(const float k)
        : m_k(k)
    {
    }

    float HuberLossFunction::Weight(const float &error) const
    {
        const float abs_error = std::fabs(error);
        return (abs_error < m_k) ? 1.0f : m_k / abs_error;
    }
} // namespace mivins
