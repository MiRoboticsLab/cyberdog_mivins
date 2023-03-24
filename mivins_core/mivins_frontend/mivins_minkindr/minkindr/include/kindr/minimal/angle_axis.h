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

#ifndef KINDR_MIN_ROTATION_ANGLE_AXIS_H_
#define KINDR_MIN_ROTATION_ANGLE_AXIS_H_

#include <ostream>

#include <Eigen/Dense>

namespace kindr
{
    namespace minimal
    {

        template <typename Scalar>
        class RotationQuaternionTemplate;

        /// \class AngleAxis
        /// \brief a minimal implementation of an angle and axis representation of
        ///        rotation
        /// This rotation takes vectors from frame B to frame A, written
        /// as \f${}_{A}\mathbf{v} = \mathbf{C}_{AB} {}_{B}\mathbf{v}\f$
        ///
        /// The angle is assumed to be in radians everywhere
        ///
        /// In code, we write:
        ///
        /// \code{.cpp}
        /// A_v = C_A_B.rotate(B_v);
        /// \endcode
        ///
        template <typename Scalar>
        class AngleAxisTemplate
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            typedef Eigen::Matrix<Scalar, 3, 1> Vector3;

            typedef Eigen::Matrix<Scalar, 4, 1> Vector4;

            typedef Eigen::AngleAxis<Scalar> Implementation;

            typedef Eigen::Matrix<Scalar, 3, 3> RotationMatrix;

            /// \brief initialize to identity.
            AngleAxisTemplate();

            /// \brief initialize from the angle and rotation axis (angle first).
            AngleAxisTemplate(Scalar angle, Scalar v1, Scalar v2, Scalar v3);

            /// \brief initialize from the angle and rotation axis.
            AngleAxisTemplate(Scalar angle, const Vector3 &axis);

            /// \brief initialize from an Eigen angleAxis.
            AngleAxisTemplate(const Implementation &angleAxis);

            /// \brief initialize from a rotation matrix.
            AngleAxisTemplate(const RotationMatrix &matrix);

            /// \brief initialize from an Eigen quaternion.
            AngleAxisTemplate(const RotationQuaternionTemplate<Scalar> &quat);

            /// \brief initialize from a angle-scaled axis vector.
            AngleAxisTemplate(const Vector3 &angleAxis);

            ~AngleAxisTemplate();

            /// \brief Returns the rotation angle.
            Scalar Angle() const;

            /// \brief Sets the rotation angle.
            void SetAngle(Scalar angle);

            /// \brief Returns the rotation axis.
            const Vector3 &Axis() const;

            /// \brief Sets the rotation axis.
            void SetAxis(const Vector3 &axis);

            /// \brief Sets the rotation axis.
            void SetAxis(Scalar v1, Scalar v2, Scalar v3);

            /// \brief get the components of the angle/axis as a vector (angle first).
            Vector4 Vector() const;

            /// \brief get a copy of the representation that is unique.
            AngleAxisTemplate<Scalar> GetUnique() const;

            /// \brief set the angle/axis to its unique representation.
            AngleAxisTemplate<Scalar> &SetUnique();

            /// \brief set the rotation to identity.
            AngleAxisTemplate<Scalar> &SetIdentity();

            /// \brief get a copy of the rotation Inverted.
            AngleAxisTemplate<Scalar> Inverse() const;

            /// \deprecated use inverse() instead.
            AngleAxisTemplate<Scalar> Inverted() const __attribute__((deprecated));

            /// \brief rotate a vector, v.
            Vector3 Rotate(const Vector3 &v) const;

            /// \brief rotate a vector, v.
            Vector4 Rotate4(const Vector4 &v) const;

            /// \brief rotate a vector, v.
            Vector3 InverseRotate(const Vector3 &v) const;

            /// \brief rotate a vector, v.
            Vector4 InverseRotate4(const Vector4 &v) const;

            /// \brief cast to the implementation type.
            Implementation &ToImplementation();

            /// \brief cast to the implementation type.
            const Implementation &ToImplementation() const;

            /// \brief get the angle between this and the other rotation.
            Scalar GetDisparityAngle(const AngleAxisTemplate<Scalar> &rhs) const;

            /// \brief enforce the unit length constraint.
            AngleAxisTemplate<Scalar> &Normalize();

            /// \brief compose two rotations.
            AngleAxisTemplate<Scalar> operator*(
                const AngleAxisTemplate<Scalar> &rhs) const;

            /// \brief assignment operator.
            AngleAxisTemplate<Scalar> &operator=(const AngleAxisTemplate<Scalar> &rhs);

            /// \brief get the rotation matrix.
            RotationMatrix GetRotationMatrix() const;

        private:
            Implementation m_C_A_B;
        };

        typedef AngleAxisTemplate<double> AngleAxis;

        template <typename Scalar>
        std::ostream &operator<<(std::ostream &out,
                                 const AngleAxisTemplate<Scalar> &rhs);

    } // namespace minimal
} // namespace kindr

#include <kindr/minimal/implementation/angle_axis_realization.h>

#endif /* KINDR_MIN_ROTATION_ANGLE_AXIS_HPP */
