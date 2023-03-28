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

// 关于角度的计算
#ifndef KINDR_MIN_ROTATION_ANGLE_AXIS_INL_H_
#define KINDR_MIN_ROTATION_ANGLE_AXIS_INL_H_
#include <kindr/minimal/angle_axis.h>
#include <kindr/minimal/rotation_quaternion.h>
#include <glog/logging.h>

namespace kindr
{
    namespace minimal
    {
        //旋转轴旋转相关
        template <typename Scalar>
        AngleAxisTemplate<Scalar>::AngleAxisTemplate() : m_C_A_B(Implementation::Identity())
        {
        }

        template <typename Scalar>
        AngleAxisTemplate<Scalar>::AngleAxisTemplate(
            Scalar w, Scalar x, Scalar y, Scalar z) : m_C_A_B(w, Vector3(x, y, z))
        {
            CHECK_NEAR(Vector3(x, y, z).squaredNorm(), static_cast<Scalar>(1.0),
                       static_cast<Scalar>(1e-4));
        }

        template <typename Scalar>
        AngleAxisTemplate<Scalar>::AngleAxisTemplate(
            Scalar angle, const typename AngleAxisTemplate<Scalar>::Vector3 &axis) : m_C_A_B(angle, axis)
        {
            CHECK_NEAR(axis.squaredNorm(), static_cast<Scalar>(1.0),
                       static_cast<Scalar>(1e-4));
        }

        template <typename Scalar>
        AngleAxisTemplate<Scalar>::AngleAxisTemplate(const Implementation &angleAxis) : m_C_A_B(angleAxis)
        {
        }

        template <typename Scalar>
        AngleAxisTemplate<Scalar>::AngleAxisTemplate(const RotationMatrix &matrix) : m_C_A_B(matrix)
        {
            // \todo furgalep Check that the matrix was good...
        }

        template <typename Scalar>
        AngleAxisTemplate<Scalar>::AngleAxisTemplate(const Vector3 &rotationVector)
        {
            const Scalar angle_rad = rotationVector.norm();
            if (angle_rad < std::numeric_limits<Scalar>::epsilon())
            {
                SetIdentity();
            }
            else
            {
                Vector3 axis = rotationVector;
                axis = axis / angle_rad;

                m_C_A_B = Implementation(angle_rad, axis);
            }
        }

        template <typename Scalar>
        AngleAxisTemplate<Scalar>::AngleAxisTemplate(
            const RotationQuaternionTemplate<Scalar> &quat) : m_C_A_B(quat.ToImplementation())
        {
        }

        template <typename Scalar>
        AngleAxisTemplate<Scalar>::~AngleAxisTemplate() {}

        template <typename Scalar>
        AngleAxisTemplate<Scalar> &AngleAxisTemplate<Scalar>::operator=(
            const AngleAxisTemplate<Scalar> &rhs)
        {
            if (this != &rhs)
            {
                m_C_A_B = rhs.m_C_A_B;
            }
            return *this;
        }

        template <typename Scalar>
        Scalar AngleAxisTemplate<Scalar>::Angle() const
        {
            return m_C_A_B.angle();
        }

        template <typename Scalar>
        void AngleAxisTemplate<Scalar>::SetAngle(Scalar angle)
        {
            m_C_A_B.angle() = angle;
        }

        template <typename Scalar>
        const typename AngleAxisTemplate<Scalar>::Vector3 &
        AngleAxisTemplate<Scalar>::Axis() const
        {
            return m_C_A_B.axis();
        }

        template <typename Scalar>
        void AngleAxisTemplate<Scalar>::SetAxis(const Vector3 &axis)
        {
            CHECK_NEAR(axis.squaredNorm(), static_cast<Scalar>(1.0),
                       static_cast<Scalar>(1e-4));
            m_C_A_B.axis() = axis;
        }

        template <typename Scalar>
        void AngleAxisTemplate<Scalar>::SetAxis(Scalar v1, Scalar v2, Scalar v3)
        {
            m_C_A_B.axis() = Vector3(v1, v2, v3);
            CHECK_NEAR(m_C_A_B.axis().squaredNorm(), static_cast<Scalar>(1.0),
                       static_cast<Scalar>(1e-4));
        }

        template <typename Scalar>
        typename AngleAxisTemplate<Scalar>::Vector4
        AngleAxisTemplate<Scalar>::Vector() const
        {
            Vector4 vector;
            vector(0) = Angle();
            vector.template tail<3>() = m_C_A_B.axis();
            return vector;
        }

        template <typename Scalar>
        AngleAxisTemplate<Scalar> AngleAxisTemplate<Scalar>::GetUnique() const
        {
            // first wraps angle into [-pi,pi)
            AngleAxisTemplate aa(fmod(Angle() + M_PI, 2 * M_PI) - M_PI, m_C_A_B.axis());
            if (aa.Angle() > 0)
            {
                return aa;
            }
            else if (aa.Angle() < 0)
            {
                if (aa.Angle() != -M_PI)
                {
                    return AngleAxisTemplate(-aa.Angle(), -aa.Axis());
                }
                else
                {   // angle == -pi, so axis must be viewed further, because -pi,axis
                    // does the same as -pi,-axis.
                    if (aa.Axis()[0] < 0)
                    {
                        return AngleAxisTemplate(-aa.Angle(), -aa.Axis());
                    }
                    else if (aa.Axis()[0] > 0)
                    {
                        return AngleAxisTemplate(-aa.Angle(), aa.Axis());
                    }
                    else
                    { // v1 == 0
                        if (aa.Axis()[1] < 0)
                        {
                            return AngleAxisTemplate(-aa.Angle(), -aa.Axis());
                        }
                        else if (aa.Axis()[1] > 0)
                        {
                            return AngleAxisTemplate(-aa.Angle(), aa.Axis());
                        }
                        else
                        { // v2 == 0
                            if (aa.Axis()[2] < 0)
                            { // v3 must be -1 or 1
                                return AngleAxisTemplate(-aa.Angle(), -aa.Axis());
                            }
                            else
                            {
                                return AngleAxisTemplate(-aa.Angle(), aa.Axis());
                            }
                        }
                    }
                }
            }
            else
            { // angle == 0
                return AngleAxisTemplate();
            }
        }

        template <typename Scalar>
        AngleAxisTemplate<Scalar> &AngleAxisTemplate<Scalar>::SetUnique()
        {
            *this = GetUnique();
            return *this;
        }

        template <typename Scalar>
        AngleAxisTemplate<Scalar> &AngleAxisTemplate<Scalar>::SetIdentity()
        {
            m_C_A_B = m_C_A_B.Identity();
            return *this;
        }

        template <typename Scalar>
        AngleAxisTemplate<Scalar> AngleAxisTemplate<Scalar>::Inverse() const
        {
            return AngleAxisTemplate(m_C_A_B.inverse());
        }

        template <typename Scalar>
        AngleAxisTemplate<Scalar> AngleAxisTemplate<Scalar>::Inverted() const
        {
            return Inverse();
        }

        template <typename Scalar>
        typename AngleAxisTemplate<Scalar>::Vector3 AngleAxisTemplate<Scalar>::Rotate(
            const AngleAxisTemplate<Scalar>::Vector3 &v) const
        {
            return m_C_A_B * v;
        }

        template <typename Scalar>
        typename AngleAxisTemplate<Scalar>::Vector4
        AngleAxisTemplate<Scalar>::Rotate4(
            const AngleAxisTemplate<Scalar>::Vector4 &v) const
        {
            AngleAxisTemplate<Scalar>::Vector4 vprime;
            vprime[3] = v[3];
            vprime.template head<3>() = m_C_A_B * v.template head<3>();
            return vprime;
        }

        template <typename Scalar>
        typename AngleAxisTemplate<Scalar>::Vector3
        AngleAxisTemplate<Scalar>::InverseRotate(
            const AngleAxisTemplate<Scalar>::Vector3 &v) const
        {
            return m_C_A_B.inverse() * v;
        }

        template <typename Scalar>
        typename AngleAxisTemplate<Scalar>::Vector4
        AngleAxisTemplate<Scalar>::InverseRotate4(
            const typename AngleAxisTemplate<Scalar>::Vector4 &v) const
        {
            Eigen::Vector4d vprime;
            vprime[3] = v[3];
            vprime.template head<3>() = m_C_A_B.inverse() * v.template head<3>();
            return vprime;
        }

        template <typename Scalar>
        typename AngleAxisTemplate<Scalar>::Implementation &
        AngleAxisTemplate<Scalar>::ToImplementation()
        {
            return m_C_A_B;
        }

        template <typename Scalar>
        const typename AngleAxisTemplate<Scalar>::Implementation &
        AngleAxisTemplate<Scalar>::ToImplementation() const
        {
            return m_C_A_B;
        }

        template <typename Scalar>
        AngleAxisTemplate<Scalar> &AngleAxisTemplate<Scalar>::Normalize()
        {
            m_C_A_B.axis().normalize();
            return *this;
        }

        template <typename Scalar>
        AngleAxisTemplate<Scalar> AngleAxisTemplate<Scalar>::operator*(
            const AngleAxisTemplate &rhs) const
        {
            return AngleAxisTemplate(Implementation(m_C_A_B * rhs.m_C_A_B));
        }

        template <typename Scalar>
        Scalar AngleAxisTemplate<Scalar>::GetDisparityAngle(
            const AngleAxisTemplate &rhs) const
        {
            return (rhs * this->Inverse()).GetUnique().Angle();
        }

        template <typename Scalar>
        std::ostream &operator<<(std::ostream &out,
                                 const AngleAxisTemplate<Scalar> &rhs)
        {
            out << rhs.Vector().transpose();
            return out;
        }

        template <typename Scalar>
        typename AngleAxisTemplate<Scalar>::RotationMatrix
        AngleAxisTemplate<Scalar>::GetRotationMatrix() const
        {
            return m_C_A_B.matrix();
        }

    } // namespace minimal
} // namespace kindr
#endif // KINDR_MIN_ROTATION_ANGLE_AXIS_INL_H_
