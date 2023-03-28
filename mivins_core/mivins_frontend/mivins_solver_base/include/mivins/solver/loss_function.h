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

#ifndef MIVINS_LOSS_FUNCTION_H_
#define MIVINS_LOSS_FUNCTION_H_

#include <vector>
#include <memory>

namespace mivins
{
    /// Scale Estimators to estimate standard deviation of a distribution of errors.
    class SigmaComputer
    {
    public:
        virtual ~SigmaComputer() = default;

        /// Errors must be absolute values!
        virtual float Compute(std::vector<float> &errors) const = 0;
    };
    typedef std::shared_ptr<SigmaComputer> SigmaComputerPtr;

    // estimates scale by computing the median absolute deviation
    class MedianSigmaComputer : public SigmaComputer
    {
    public:
        using SigmaComputer::SigmaComputer;
        virtual ~MedianSigmaComputer() = default;
        virtual float Compute(std::vector<float> &errors) const;
    };

    // estimates scale by computing the standard deviation
    class NormalDistributionSigmaComputer : public SigmaComputer
    {
    public:
        using SigmaComputer::SigmaComputer;
        virtual ~NormalDistributionSigmaComputer() = default;
        virtual float Compute(std::vector<float> &errors) const;

    private:
    };

    /// Weight-Functions for M-Estimators
    /// http://research.microsoft.com/en-us/um/people/zhang/inria/publis/tutorial-estim/node24.html
    class LossFunction
    {
    public:
        LossFunction() = default;
        virtual ~LossFunction() = default;
        virtual float Weight(const float &error) const = 0;
    };
    typedef std::shared_ptr<LossFunction> LossFunctionPtr;

    class TukeyLossFunction : public LossFunction
    {
    public:
        TukeyLossFunction(const float b = 4.6851f);
        virtual ~TukeyLossFunction() = default;
        virtual float Weight(const float &error) const;

    private:
        float m_b_square_;
    };

    class HuberLossFunction : public LossFunction
    {
    public:
        HuberLossFunction(const float k = 1.345f);
        virtual ~HuberLossFunction() = default;
        virtual float Weight(const float &error) const;

    private:
        float m_k;
    };
} // namespace mivins

#endif // MIVINS_LOSS_FUNCTION_H_
