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

#ifndef KINDR_MINIMAL_QUAT_TRANSFORMATION_H_INL_
#define KINDR_MINIMAL_QUAT_TRANSFORMATION_H_INL_
#include <kindr/minimal/quat_transformation.h>

#include <glog/logging.h>
//位姿变换
namespace kindr
{
    namespace minimal
    {

        template <typename Scalar>
        QuatTransformationTemplate<Scalar>::QuatTransformationTemplate()
        {
            setIdentity();
        }

        template <typename Scalar>
        QuatTransformationTemplate<Scalar>::QuatTransformationTemplate(
            const RotationQuaternionTemplate<Scalar> &q_A_B, const Position &A_t_A_B) : m_q_A_B(q_A_B),
                                                                                        m_A_t_A_B(A_t_A_B)
        {
        }

        template <typename Scalar>
        QuatTransformationTemplate<Scalar>::QuatTransformationTemplate(
            const typename Rotation::Implementation &q_A_B,
            const Position &A_t_A_B) : m_q_A_B(q_A_B),
                                       m_A_t_A_B(A_t_A_B)
        {
        }

        template <typename Scalar>
        QuatTransformationTemplate<Scalar>::QuatTransformationTemplate(
            const Position &A_t_A_B, const RotationQuaternionTemplate<Scalar> &q_A_B) : m_q_A_B(q_A_B),
                                                                                        m_A_t_A_B(A_t_A_B)
        {
        }

        template <typename Scalar>
        QuatTransformationTemplate<Scalar>::QuatTransformationTemplate(
            const Position &A_t_A_B, const typename Rotation::Implementation &q_A_B) : m_q_A_B(q_A_B),
                                                                                       m_A_t_A_B(A_t_A_B)
        {
        }

        template <typename Scalar>
        QuatTransformationTemplate<Scalar>::QuatTransformationTemplate(
            const TransformationMatrix &T) : m_q_A_B(T.template topLeftCorner<3, 3>().eval()),
                                             m_A_t_A_B(T.template topRightCorner<3, 1>().eval())
        {
        }

        template <typename Scalar>
        QuatTransformationTemplate<Scalar>::QuatTransformationTemplate(
            const QuatTransformationTemplate<Scalar>::Vector6 &x_t_r) : m_q_A_B(x_t_r.template tail<3>().eval()),
                                                                        m_A_t_A_B(x_t_r.template head<3>().eval())
        {
        }

        template <typename Scalar>
        QuatTransformationTemplate<Scalar>::~QuatTransformationTemplate()
        {
        }

        template <typename Scalar>
        void QuatTransformationTemplate<Scalar>::setIdentity()
        {
            m_q_A_B.SetIdentity();
            m_A_t_A_B.setZero();
        }

        template <typename Scalar>
        typename QuatTransformationTemplate<Scalar>::Position &
        QuatTransformationTemplate<Scalar>::GetPosition()
        {
            return m_A_t_A_B;
        }

        template <typename Scalar>
        const typename QuatTransformationTemplate<Scalar>::Position &
        QuatTransformationTemplate<Scalar>::GetPosition() const
        {
            return m_A_t_A_B;
        }

        template <typename Scalar>
        typename QuatTransformationTemplate<Scalar>::Rotation &
        QuatTransformationTemplate<Scalar>::GetRotation()
        {
            return m_q_A_B;
        }

        template <typename Scalar>
        const typename QuatTransformationTemplate<Scalar>::Rotation &
        QuatTransformationTemplate<Scalar>::GetRotation() const
        {
            return m_q_A_B;
        }

        template <typename Scalar>
        const Eigen::Quaternion<Scalar> &
        QuatTransformationTemplate<Scalar>::GetEigenQuaternion() const
        {
            return GetRotation().ToImplementation();
        }

        template <typename Scalar>
        typename QuatTransformationTemplate<Scalar>::TransformationMatrix
        QuatTransformationTemplate<Scalar>::GetTransformationMatrix() const
        {
            TransformationMatrix transformation_matrix;
            transformation_matrix.setIdentity();
            transformation_matrix.template topLeftCorner<3, 3>() =
                m_q_A_B.GetRotationMatrix();
            transformation_matrix.template topRightCorner<3, 1>() = m_A_t_A_B;
            return transformation_matrix;
        }

        template <typename Scalar>
        Eigen::Matrix<Scalar, 7, 1>
        QuatTransformationTemplate<Scalar>::AsVector() const
        {
            return (Eigen::Matrix<Scalar, 7, 1>() << m_q_A_B.w(), m_q_A_B.x(), m_q_A_B.y(), m_q_A_B.z(), m_A_t_A_B).finished();
        }

        template <typename Scalar>
        typename QuatTransformationTemplate<Scalar>::RotationMatrix
        QuatTransformationTemplate<Scalar>::GetRotationMatrix() const
        {
            return m_q_A_B.GetRotationMatrix();
        }

        template <typename Scalar>
        QuatTransformationTemplate<Scalar>
        QuatTransformationTemplate<Scalar>::operator*(
            const QuatTransformationTemplate<Scalar> &rhs) const
        {
            return QuatTransformationTemplate<Scalar>(m_q_A_B * rhs.m_q_A_B, m_A_t_A_B +
                                                                                 m_q_A_B.Rotate(rhs.m_A_t_A_B));
        }

        template <typename Scalar>
        typename QuatTransformationTemplate<Scalar>::Vector3
        QuatTransformationTemplate<Scalar>::Transform(
            const typename QuatTransformationTemplate<Scalar>::Vector3 &rhs) const
        {
            return m_q_A_B.Rotate(rhs) + m_A_t_A_B;
        }

        template <typename Scalar>
        typename QuatTransformationTemplate<Scalar>::Matrix3X
        QuatTransformationTemplate<Scalar>::TransformVectorized(
            const typename QuatTransformationTemplate<Scalar>::Matrix3X &rhs) const
        {
            CHECK_GT(rhs.cols(), 0);
            return m_q_A_B.rotateVectorized(rhs).colwise() + m_A_t_A_B;
        }

        template <typename Scalar>
        typename QuatTransformationTemplate<Scalar>::Vector3
        QuatTransformationTemplate<Scalar>::operator*(
            const typename QuatTransformationTemplate<Scalar>::Vector3 &rhs) const
        {
            return Transform(rhs);
        }

        template <typename Scalar>
        typename QuatTransformationTemplate<Scalar>::Vector4
        QuatTransformationTemplate<Scalar>::Transform4(
            const typename QuatTransformationTemplate<Scalar>::Vector4 &rhs) const
        {
            QuatTransformationTemplate<Scalar>::Vector4 rval;
            rval[3] = rhs[3];
            rval.template head<3>() =
                m_q_A_B.rotate(rhs.template head<3>()) + rhs[3] * m_A_t_A_B;
            return rval;
        }

        template <typename Scalar>
        typename QuatTransformationTemplate<Scalar>::Vector3
        QuatTransformationTemplate<Scalar>::InverseTransform(
            const Vector3 &rhs) const
        {
            return m_q_A_B.InverseRotate(rhs - m_A_t_A_B);
        }

        template <typename Scalar>
        typename QuatTransformationTemplate<Scalar>::Vector4
        QuatTransformationTemplate<Scalar>::InverseTransform4(
            const typename QuatTransformationTemplate<Scalar>::Vector4 &rhs) const
        {
            typename QuatTransformationTemplate<Scalar>::Vector4 rval;
            rval.template head<3>() = m_q_A_B.InverseRotate(rhs.template head<3>() -
                                                            m_A_t_A_B * rhs[3]);
            rval[3] = rhs[3];
            return rval;
        }

        template <typename Scalar>
        QuatTransformationTemplate<Scalar>
        QuatTransformationTemplate<Scalar>::Inverse() const
        {
            return QuatTransformationTemplate<Scalar>(m_q_A_B.Inverse(), -m_q_A_B.InverseRotate(m_A_t_A_B));
        }

        template <typename Scalar>
        QuatTransformationTemplate<Scalar>
        QuatTransformationTemplate<Scalar>::Inverted() const
        {
            return Inverse();
        }

        template <typename Scalar>
        typename QuatTransformationTemplate<Scalar>::Vector6
        QuatTransformationTemplate<Scalar>::Log() const
        {
            return Log(*this);
        }

        template <typename Scalar>
        QuatTransformationTemplate<Scalar> QuatTransformationTemplate<Scalar>::Exp(const Vector6 &vec)
        {
            return QuatTransformationTemplate<Scalar>(vec);
        }

        template <typename Scalar>
        typename QuatTransformationTemplate<Scalar>::Vector6
        QuatTransformationTemplate<Scalar>::Log(const QuatTransformationTemplate<Scalar> &T)
        {
            AngleAxisTemplate<Scalar> angleaxis(T.m_q_A_B);
            return (Vector6() << T.m_A_t_A_B, T.m_q_A_B.Log()).finished();
        }

        template <typename Scalar>
        QuatTransformationTemplate<Scalar> &
        QuatTransformationTemplate<Scalar>::SetRandom()
        {
            m_q_A_B.SetRandom();
            m_A_t_A_B.SetRandom();
            return *this;
        }

        template <typename Scalar>
        QuatTransformationTemplate<Scalar> &
        QuatTransformationTemplate<Scalar>::SetRandom(Scalar norm_translation)
        {
            SetRandom();
            m_A_t_A_B.normalize();
            m_A_t_A_B *= norm_translation;
            return *this;
        }

        template <typename Scalar>
        QuatTransformationTemplate<Scalar> &
        QuatTransformationTemplate<Scalar>::SetRandom(Scalar norm_translation, Scalar angle_rad)
        {
            m_q_A_B.SetRandom(angle_rad);
            m_A_t_A_B.SetRandom().normalize();
            m_A_t_A_B *= norm_translation;
            return *this;
        }

        template <typename Scalar>
        std::ostream &operator<<(std::ostream &out,
                                 const QuatTransformationTemplate<Scalar> &pose)
        {
            out << pose.GetTransformationMatrix();
            return out;
        }

        template <typename Scalar>
        bool QuatTransformationTemplate<Scalar>::operator==(
            const QuatTransformationTemplate<Scalar> &rhs) const
        {
            return m_q_A_B == rhs.m_q_A_B && m_A_t_A_B == rhs.m_A_t_A_B;
        }

        template <typename Scalar>
        QuatTransformationTemplate<Scalar> QuatTransformationTemplate<
            Scalar>::ConstructAndRenormalizeRotation(const TransformationMatrix &T)
        {
            return QuatTransformationTemplate<Scalar>(
                Rotation::constructAndRenormalize(
                    T.template topLeftCorner<3, 3>().eval()),
                T.template topRightCorner<3, 1>().eval());
        }

        template <typename Scalar>
        template <typename ScalarAfterCast>
        QuatTransformationTemplate<ScalarAfterCast>
        QuatTransformationTemplate<Scalar>::Cast() const
        {
            return QuatTransformationTemplate<ScalarAfterCast>(
                GetRotation().template cast<ScalarAfterCast>(),
                GetPosition().template cast<ScalarAfterCast>());
        }

        template <typename Scalar>
        QuatTransformationTemplate<Scalar> InterpolateComponentwise(
            const QuatTransformationTemplate<Scalar> &T_a,
            const QuatTransformationTemplate<Scalar> &T_b, const double lambda)
        {
            CHECK_GE(lambda, 0.0);
            CHECK_LE(lambda, 1.0);
            const PositionTemplate<Scalar> p_int =
                T_a.GetPosition() + lambda * (T_b.GetPosition() - T_a.GetPosition());
            const Eigen::Quaternion<Scalar> q_int =
                T_a.GetEigenQuaternion().slerp(lambda, T_b.GetEigenQuaternion());
            return QuatTransformationTemplate<Scalar>(q_int, p_int);
        }

    } // namespace minimal
} // namespace kindr
#endif // KINDR_MINIMAL_QUAT_TRANSFORMATION_H_INL_
