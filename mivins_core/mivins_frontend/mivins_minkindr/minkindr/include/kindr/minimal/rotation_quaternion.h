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

//Rotation
#ifndef KINDR_MIN_ROTATION_QUATERNION_H_
#define KINDR_MIN_ROTATION_QUATERNION_H_

#include <ostream>

#include <Eigen/Dense>

namespace kindr
{
    namespace minimal
    {

        template <typename Scalar>
        class AngleAxisTemplate;

        /// \class RotationQuaternion
        /// \brief a minimal implementation of a passive Hamiltonian rotation
        ///        (unit-length) quaternion
        ///
        /// This rotation takes vectors from frame B to frame A, written
        /// as \f${}_{A}\mathbf{v} = \mathbf{C}_{AB} {}_{B}\mathbf{v}\f$
        ///
        /// In code, we write:
        ///
        /// \code{.cpp}
        /// A_v = q_A_B.rotate(B_v);
        /// \endcode
        ///
        template <typename Scalar>
        class RotationQuaternionTemplate
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            typedef Eigen::Matrix<Scalar, 3, 1> Vector3;

            typedef Vector3 Imaginary;

            typedef Eigen::Matrix<Scalar, 4, 1> Vector4;

            typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Matrix3X;

            typedef Eigen::Quaternion<Scalar> Implementation;

            typedef Eigen::Matrix<Scalar, 3, 3> RotationMatrix;

            /// \brief initialize to identity.
            RotationQuaternionTemplate();

            /// \brief initialize from angle scaled axis.
            explicit RotationQuaternionTemplate(const Vector3 &angle_scaled_axis);

            /// \brief initialize from real and imaginary components (real first).
            explicit RotationQuaternionTemplate(Scalar w, Scalar x, Scalar y, Scalar z);

            /// \brief initialize from real and imaginary components.
            explicit RotationQuaternionTemplate(Scalar real, const Imaginary &imaginary);

            /// \brief initialize from an Eigen quaternion.
            explicit RotationQuaternionTemplate(const Implementation &quaternion);

            /// \brief initialize from a rotation matrix.
            explicit RotationQuaternionTemplate(const RotationMatrix &matrix);

            /// \brief take an approximate rotation matrix, recover the closest matrix
            /// in SO(3) and construct.
            static RotationQuaternionTemplate<Scalar> FromApproximateRotationMatrix(
                const RotationMatrix &matrix);

            /// \brief initialize from an AngleAxis.
            explicit RotationQuaternionTemplate(
                const AngleAxisTemplate<Scalar> &angleAxis);

            ~RotationQuaternionTemplate();

            /// \brief the real component of the quaternion.
            Scalar w() const;
            /// \brief the first imaginary component of the quaternion.
            Scalar x() const;
            /// \brief the second imaginary component of the quaternion.
            Scalar y() const;
            /// \brief the third imaginary component of the quaternion.
            Scalar z() const;

            /// \brief the imaginary components of the quaterion.
            Imaginary ImaginaryComponents() const;

            /// \brief get the components of the quaternion as a vector (real first).
            Vector4 Vector() const;

            /// \brief set the quaternion by its values (real, imaginary).
            void SetValues(Scalar w, Scalar x, Scalar y, Scalar z);

            /// \brief set the quaternion by its real and imaginary parts.
            void SetParts(Scalar real, const Imaginary &imag);

            /// \brief get a copy of the representation that is unique.
            RotationQuaternionTemplate<Scalar> GetUnique() const;

            /// \brief set the quaternion to its unique representation.
            RotationQuaternionTemplate<Scalar> &SetUnique();

            /// \brief set the quaternion to identity.
            RotationQuaternionTemplate<Scalar> &SetIdentity();

            /// \brief set to random rotation.
            RotationQuaternionTemplate<Scalar> &SetRandom();

            /// \brief set to random rotation with a given angle.
            RotationQuaternionTemplate<Scalar> &SetRandom(Scalar angle_rad);

            /// \brief get a copy of the quaternion inverted.
            RotationQuaternionTemplate<Scalar> Inverse() const;

            /// \deprecated use inverse instead.
            RotationQuaternionTemplate<Scalar> Inverted() const __attribute__((deprecated));

            /// \brief get a copy of the conjugate of the quaternion.
            RotationQuaternionTemplate<Scalar> Conjugated() const;

            /// \brief rotate a vector, v.
            Vector3 Rotate(const Vector3 &v) const;

            /// \brief rotate vectors v.
            Matrix3X RotateVectorized(const Matrix3X &v) const;

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

            /// \brief get the norm of the quaternion.
            Scalar Norm() const;

            /// \brief get the squared norm of the quaternion.
            Scalar SquaredNorm() const;

            /// \brief get the angle [rad] between this and the other quaternion.
            Scalar GetDisparityAngle(const RotationQuaternionTemplate<Scalar> &rhs) const;

            /// \brief get the angle [rad] between this and the angle axis.
            Scalar GetDisparityAngle(const AngleAxisTemplate<Scalar> &rhs) const;

            /// \brief enforce the unit length constraint.
            RotationQuaternionTemplate<Scalar> &Normalize();

            /// \brief compose two quaternions.
            RotationQuaternionTemplate<Scalar> operator*(
                const RotationQuaternionTemplate<Scalar> &rhs) const;

            /// \brief compose quaternion and angle axis.
            RotationQuaternionTemplate<Scalar> operator*(
                const AngleAxisTemplate<Scalar> &rhs) const;

            /// \brief assignment operator.
            RotationQuaternionTemplate<Scalar> &operator=(
                const RotationQuaternionTemplate<Scalar> &rhs);

            /// \brief get the rotation matrix.
            RotationMatrix GetRotationMatrix() const;

            /// \brief check for binary equality.
            bool operator==(const RotationQuaternionTemplate<Scalar> &rhs) const
            {
                return Vector() == rhs.vector();
            }

            // Compute the matrix log of the quaternion.
            static Vector3 Log(const RotationQuaternionTemplate<Scalar> &q);

            // Compute the matrix exponential of the quaternion.
            static RotationQuaternionTemplate<Scalar> Exp(const Vector3 &dx);

            Vector3 Log() const;

            /// \brief Check the validity of a rotation matrix.
            static bool IsValidRotationMatrix(const RotationMatrix &matrix);
            static bool IsValidRotationMatrix(const RotationMatrix &matrix,
                                              const Scalar threshold);

            /// \brief Factory to construct a RotationQuaternionTemplate from a near
            ///        orthonormal rotation matrix.
            inline static RotationQuaternionTemplate<Scalar> ConstructAndRenormalize(
                const RotationMatrix &R)
            {
                return RotationQuaternionTemplate<Scalar>(Implementation(R).normalized());
            }

            /// \brief cast scalar elements to another type
            template <typename ScalarAfterCast>
            RotationQuaternionTemplate<ScalarAfterCast> Cast() const;

        private:
            void NormalizationHelper(Implementation *quaternion) const;

            Implementation m_q_A_B;
        };

        typedef RotationQuaternionTemplate<double> RotationQuaternion;

        template <typename Scalar>
        std::ostream &operator<<(std::ostream &out,
                                 const RotationQuaternionTemplate<Scalar> &rhs);

    } // namespace minimal
} // namespace kindr

#include <kindr/minimal/implementation/rotation_quaternion_realization.h>

#endif // KINDR_MIN_ROTATION_QUATERNION_H_
