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

#ifndef KINDR_MINIMAL_IMPLEMENTATION_TRANSFORM_2D_INL_H_
#define KINDR_MINIMAL_IMPLEMENTATION_TRANSFORM_2D_INL_H_

#include <cmath>
#include <limits>

#include <glog/logging.h>

namespace kindr
{
    namespace minimal
    {
        //2d trans+rotate变换
        template <typename Scalar>
        Transformation2DTemplate<Scalar>::Transformation2DTemplate()
        {
            setIdentity();
        }

        template <typename Scalar>
        Transformation2DTemplate<Scalar>::Transformation2DTemplate(
            const Rotation r_A_B, const Position &A_t_A_B)
            : m_r_A_B(r_A_B), m_A_t_A_B(A_t_A_B) {}

        template <typename Scalar>
        Transformation2DTemplate<Scalar>::Transformation2DTemplate(
            const TransformationMatrix &T)
            : Transformation2DTemplate<Scalar>(
                  Rotation2D().fromRotationMatrix(T.template topLeftCorner<2, 2>().eval()),
                  T.template topRightCorner<2, 1>().eval())
        {
            constexpr Scalar kEpsilon = std::numeric_limits<Scalar>::epsilon();
            CHECK_LE((T(2, 2) - static_cast<Scalar>(1.0)), kEpsilon);
            const Eigen::Matrix<Scalar, 2, 2> rotation_matrix =
                T.template topLeftCorner<2, 2>().eval();
            CHECK_NEAR(rotation_matrix.determinant(), static_cast<Scalar>(1.0), kEpsilon);
        }

        template <typename Scalar>
        Transformation2DTemplate<Scalar>::~Transformation2DTemplate() {}

        template <typename Scalar>
        void Transformation2DTemplate<Scalar>::SetIdentity()
        {
            m_r_A_B = Rotation2D(static_cast<Scalar>(0.0));
            m_A_t_A_B.setZero();
        }

        template <typename Scalar>
        typename Transformation2DTemplate<Scalar>::Position &
        Transformation2DTemplate<Scalar>::SetPosition()
        {
            return m_A_t_A_B;
        }

        template <typename Scalar>
        const typename Transformation2DTemplate<Scalar>::Position &
        Transformation2DTemplate<Scalar>::GetPosition() const
        {
            return m_A_t_A_B;
        }

        template <typename Scalar>
        typename Transformation2DTemplate<Scalar>::Rotation &
        Transformation2DTemplate<Scalar>::GetRotation()
        {
            return m_r_A_B;
        }

        template <typename Scalar>
        const typename Transformation2DTemplate<Scalar>::Rotation &
        Transformation2DTemplate<Scalar>::GetRotation() const
        {
            return m_r_A_B;
        }

        template <typename Scalar>
        typename Transformation2DTemplate<Scalar>::RotationMatrix
        Transformation2DTemplate<Scalar>::GetRotationMatrix() const
        {
            return m_r_A_B.toRotationMatrix();
        }

        template <typename Scalar>
        typename Transformation2DTemplate<Scalar>::TransformationMatrix
        Transformation2DTemplate<Scalar>::GetTransformationMatrix() const
        {
            TransformationMatrix transformation_matrix;
            transformation_matrix.template topLeftCorner<2, 2>() =
                m_r_A_B.toRotationMatrix();
            transformation_matrix.template topRightCorner<2, 1>() = m_A_t_A_B;
            transformation_matrix.template bottomRows<1>() =
                (Eigen::Matrix<Scalar, 1, 3>() << static_cast<Scalar>(0.0),
                 static_cast<Scalar>(0.0), static_cast<Scalar>(1.0))
                    .finished();
            return transformation_matrix;
        }

        template <typename Scalar>
        Eigen::Matrix<Scalar, 3, 1> Transformation2DTemplate<Scalar>::AsVector() const
        {
            return (Eigen::Matrix<Scalar, 3, 1>() << m_r_A_B.angle(), m_A_t_A_B).finished();
        }

        template <typename Scalar>
        Transformation2DTemplate<Scalar> Transformation2DTemplate<Scalar>::operator*(
            const Transformation2DTemplate<Scalar> &rhs) const
        {
            return Transformation2DTemplate<Scalar>(
                m_r_A_B * rhs.m_r_A_B, m_A_t_A_B + m_r_A_B * rhs.m_A_t_A_B);
        }

        template <typename Scalar>
        typename Transformation2DTemplate<Scalar>::Vector2
        Transformation2DTemplate<Scalar>::operator*(const Vector2 &rhs) const
        {
            return Transform(rhs);
        }

        template <typename Scalar>
        typename Transformation2DTemplate<Scalar>::Vector2
        Transformation2DTemplate<Scalar>::Transform(const Vector2 &rhs) const
        {
            return m_r_A_B * rhs + m_A_t_A_B;
        }

        template <typename Scalar>
        typename Transformation2DTemplate<Scalar>::Matrix2X
        Transformation2DTemplate<Scalar>::TransformVectorized(
            const Matrix2X &rhs) const
        {
            return (m_r_A_B.toRotationMatrix() * rhs).colwise() + m_A_t_A_B;
        }

        template <typename Scalar>
        Transformation2DTemplate<Scalar> Transformation2DTemplate<Scalar>::Inverse()
            const
        {
            return Transformation2DTemplate<Scalar>(
                m_r_A_B.inverse(), -(m_r_A_B.inverse() * m_A_t_A_B));
        }

        template <typename Scalar>
        bool Transformation2DTemplate<Scalar>::operator==(
            const Transformation2DTemplate<Scalar> &rhs) const
        {
            return m_r_A_B.angle() == rhs.m_r_A_B.angle() && m_A_t_A_B == rhs.m_A_t_A_B;
        }

        template <typename Scalar>
        bool Transformation2DTemplate<Scalar>::operator!=(
            const Transformation2DTemplate<Scalar> &rhs) const
        {
            return !(*this == rhs);
        }

        template <typename Scalar>
        std::ostream &operator<<(std::ostream &out,
                                 const Transformation2DTemplate<Scalar> &rhs)
        {
            out << "[" << rhs.GetRotation().angle() << ", ["
                << rhs.GetPosition().transpose() << "]]";
            return out;
        }

        template <typename Scalar>
        template <typename ScalarAfterCast>
        Transformation2DTemplate<ScalarAfterCast>
        Transformation2DTemplate<Scalar>::Cast() const
        {
            return Transformation2DTemplate<ScalarAfterCast>(
                getRotation().template cast<ScalarAfterCast>(),
                getPosition().template cast<ScalarAfterCast>());
        }

    } // namespace minimal
} // namespace kindr

#endif // KINDR_MINIMAL_IMPLEMENTATION_TRANSFORM_2D_INL_H_
