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

#ifndef KINDR_MINIMAL_IMPLEMENTATION_QUAT_SIM_TRANSFORM_INL_H_
#define KINDR_MINIMAL_IMPLEMENTATION_QUAT_SIM_TRANSFORM_INL_H_

#include <glog/logging.h>

namespace kindr
{
    namespace minimal
    {
        //相似变换  no use
        template <typename Scalar>
        QuatSimTransformTemplate<Scalar>::QuatSimTransformTemplate() : scale_A_B_(1.) {}

        template <typename Scalar>
        QuatSimTransformTemplate<Scalar>::QuatSimTransformTemplate(
            const Transform &T_A_B, const Scalar scale_A_B)
            : T_A_B_(T_A_B), scale_A_B_(scale_A_B)
        {
            CHECK_GT(scale_A_B, 0.);
        }

        template <typename Scalar>
        QuatSimTransformTemplate<Scalar>::QuatSimTransformTemplate(
            const Vector7 &log_vector)
            : T_A_B_(Eigen::Matrix<Scalar, 6, 1>(log_vector.template head<6>())),
              scale_A_B_(log_vector(6))
        {
            CHECK_GT(scale_A_B_, 0.);
        }

        template <typename Scalar>
        typename QuatSimTransformTemplate<Scalar>::Vector3
        QuatSimTransformTemplate<Scalar>::operator*(const Vector3 &rhs) const
        {
            return T_A_B_ * (scale_A_B_ * rhs);
        }

        template <typename Scalar>
        typename QuatSimTransformTemplate<Scalar>::Matrix3X
        QuatSimTransformTemplate<Scalar>::operator*(const Matrix3X &rhs) const
        {
            return T_A_B_.transformVectorized(scale_A_B_ * rhs);
        }

        template <typename Scalar>
        QuatSimTransformTemplate<Scalar> QuatSimTransformTemplate<Scalar>::operator*(
            const QuatSimTransformTemplate<Scalar> &rhs) const
        {
            // Rl*sl tl         Rr*sr tr         Rl*sl*Rr*sr Rl*sl*tr+tl
            // 0     1     *    0     1     =    0           1
            //
            // -> R = Rl*Rr;  t = Rl*sl*tr+tl = Siml*tr;  s = sl*sr
            return QuatSimTransformTemplate<Scalar>(
                QuatTransformationTemplate<Scalar>(
                    T_A_B_.GetRotation() * rhs.T_A_B_.GetRotation(),
                    (*this) * rhs.T_A_B_.GetPosition()),
                scale_A_B_ * rhs.scale_A_B_);
        }

        template <typename Scalar>
        QuatSimTransformTemplate<Scalar> QuatSimTransformTemplate<Scalar>::operator*(
            const QuatTransformationTemplate<Scalar> &rhs) const
        {
            return operator*(QuatSimTransformTemplate<Scalar>(rhs, 1.));
        }

        template <typename Scalar>
        QuatSimTransformTemplate<Scalar>
        QuatSimTransformTemplate<Scalar>::Inverse() const
        {
            return QuatSimTransformTemplate<Scalar>(
                QuatTransformationTemplate<Scalar>(
                    T_A_B_.GetRotation().inverse(), -T_A_B_.GetRotation().InverseRotate(
                                                        T_A_B_.GetPosition() / scale_A_B_)),
                1. / scale_A_B_);
        }

        template <typename Scalar>
        inline Eigen::Matrix<Scalar, 7, 1>
        QuatSimTransformTemplate<Scalar>::Log() const
        {
            Eigen::Matrix<Scalar, 7, 1> result;
            result << T_A_B_.log(), scale_A_B_;
            return result;
        }

        template <typename Scalar>
        Eigen::Matrix<Scalar, 4, 4>
        QuatSimTransformTemplate<Scalar>::GetTransformationMatrix() const
        {
            Eigen::Matrix<Scalar, 4, 4> result = T_A_B_.GetTransformationMatrix();
            result.template topLeftCorner<3, 3>() *= scale_A_B_;
            return result;
        }

        template <typename Scalar>
        inline QuatSimTransformTemplate<Scalar> operator*(
            const QuatTransformationTemplate<Scalar> &lhs,
            const QuatSimTransformTemplate<Scalar> &rhs)
        {
            return QuatSimTransformTemplate<Scalar>(lhs, 1.) * rhs;
        }

        template <typename Scalar>
        std::ostream &operator<<(std::ostream &out,
                                 const QuatSimTransformTemplate<Scalar> &sim_3)
        {
            out << "Transform:" << std::endl
                << sim_3.T_A_B_.GetTransformationMatrix() << std::endl;
            out << "Scale: " << sim_3.scale_A_B_;
            return out;
        }

    } // namespace minimal
} // namespace kindr

#endif // KINDR_MINIMAL_IMPLEMENTATION_QUAT_SIM_TRANSFORM_INL_H_
