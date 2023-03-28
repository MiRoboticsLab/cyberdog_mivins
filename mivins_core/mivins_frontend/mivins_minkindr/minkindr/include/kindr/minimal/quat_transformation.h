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

#ifndef KINDR_MINIMAL_QUAT_TRANSFORMATION_H_
#define KINDR_MINIMAL_QUAT_TRANSFORMATION_H_

#include <ostream>

#include <kindr/minimal/rotation_quaternion.h>
#include <kindr/minimal/position.h>
//trans+rotate
namespace kindr
{
    namespace minimal
    {

        /// \class QuatTransformation
        /// \brief A frame transformation built from a quaternion and a point
        ///
        /// This transformation takes points from frame B to frame A, written
        /// as \f${}_{A}\mathbf{p} = \mathbf{T}_{AB} {}_{B}\mathbf{p}\f$
        ///
        /// In code, we write:
        ///
        /// \code{.cpp}
        /// A_p = T_A_B.transform(B_p);
        /// \endcode
        ///
        template <typename Scalar>
        class QuatTransformationTemplate
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
            typedef Eigen::Matrix<Scalar, 4, 1> Vector4;
            typedef Eigen::Matrix<Scalar, 6, 1> Vector6;

            typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Matrix3X;

            typedef PositionTemplate<Scalar> Position;
            typedef RotationQuaternionTemplate<Scalar> Rotation;
            typedef Eigen::Matrix<Scalar, 3, 3> RotationMatrix;
            typedef Eigen::Matrix<Scalar, 4, 4> TransformationMatrix;

            /// \brief Constructor of identity transformation.
            QuatTransformationTemplate();

            explicit QuatTransformationTemplate(
                const RotationQuaternionTemplate<Scalar> &q_A_B, const Position &A_t_A_B);
            explicit QuatTransformationTemplate(
                const typename Rotation::Implementation &q_A_B, const Position &A_t_A_B);

            explicit QuatTransformationTemplate(
                const Position &A_t_A_B, const Rotation &q_A_B);
            explicit QuatTransformationTemplate(
                const Position &A_t_A_B, const typename Rotation::Implementation &q_A_B);

            explicit QuatTransformationTemplate(const TransformationMatrix &T);

            /// \brief a constructor based on the exponential map.
            /// translational part in the first 3 dimensions,
            /// rotational part in the last 3 dimensions.
            QuatTransformationTemplate(const Vector6 &x_t_r);

            ~QuatTransformationTemplate();

            void setIdentity();

            /// \brief set to random transformation.
            QuatTransformationTemplate<Scalar> &SetRandom();

            /// \brief set to random transformation with a given translation norm.
            QuatTransformationTemplate<Scalar> &SetRandom(Scalar norm_translation);

            /// \brief set to random transformation with a given translation norm and rotation angle.
            QuatTransformationTemplate<Scalar> &SetRandom(Scalar norm_translation, Scalar angle_rad);

            /// \brief get the position component.
            Position &GetPosition();

            /// \brief get the position component.
            const Position &GetPosition() const;

            /// \brief get the rotation component.
            Rotation &GetRotation();

            /// \brief get the rotation component.
            const Rotation &GetRotation() const;

            /// \brief get the rotation component as an Eigen Quaternion directly.
            const Eigen::Quaternion<Scalar> &GetEigenQuaternion() const;

            /// \brief get the transformation matrix.
            TransformationMatrix GetTransformationMatrix() const;

            /// \brief get the rotation matrix.
            RotationMatrix GetRotationMatrix() const;

            /// \brief get the quaternion of rotation and the position as a vector.
            ///  [w x y z, x y z]
            Eigen::Matrix<Scalar, 7, 1> AsVector() const;

            /// \brief compose two transformations.
            QuatTransformationTemplate<Scalar> operator*(
                const QuatTransformationTemplate<Scalar> &rhs) const;

            /// \brief transform a point.
            Vector3 operator*(const Vector3 &rhs) const;

            /// \brief transform a point.
            Vector3 Transform(const Vector3 &rhs) const;

            /// \brief transform points.
            Matrix3X TransformVectorized(const Matrix3X &rhs) const;

            /// \brief transform a point.
            Vector4 Transform4(const Vector4 &rhs) const;

            /// \brief transform a point by the inverse.
            Vector3 InverseTransform(const Vector3 &rhs) const;

            /// \brief transform a point by the inverse.
            Vector4 InverseTransform4(const Vector4 &rhs) const;

            /// \brief get the logarithmic map of the transformation
            /// note: this is the log map of SO(3)xR(3) and not SE(3)
            /// \return vector form of log map with first 3 components the translational
            ///         part and the last three the rotational part.
            Vector6 Log() const;

            /// \brief get the exponential map of the parameters, resulting in a valid
            /// transformation note: this is the exp map of SO(3)xR(3) and not SE(3)
            /// \param[in] vec vector form of log map with first 3 components the translational
            ///                part and the last three the rotational part.
            /// \return The corresponding Transformation.
            static QuatTransformationTemplate<Scalar> Exp(const Vector6 &vec);

            /// \brief get the logarithmic map of the transformation
            /// note: this is the log map of SO(3)xR(3) and not SE(3)
            /// \return vector form of log map with first 3 components the translational
            ///         part and the last three the rotational part.
            static Vector6 Log(const QuatTransformationTemplate<Scalar> &vec);

            /// \brief return a copy of the transformation inverted.
            QuatTransformationTemplate<Scalar> Inverse() const;

            /// \deprecated use inverse() instead.
            QuatTransformationTemplate<Scalar> Inverted() const __attribute__((deprecated));

            /// \brief check for binary equality.
            bool operator==(const QuatTransformationTemplate<Scalar> &rhs) const;

            /// \brief Factory to construct a QuatTransformTemplate from a transformation
            ///        matrix with a near orthonormal rotation matrix.
            static QuatTransformationTemplate<Scalar>
            ConstructAndRenormalizeRotation(const TransformationMatrix &T);

            /// \brief cast scalar elements to another type
            template <typename ScalarAfterCast>
            QuatTransformationTemplate<ScalarAfterCast> Cast() const;

        private:
            /// The quaternion that takes vectors from B to A.
            ///
            /// \code{.cpp}
            /// A_v = m_q_A_B.rotate(B_v);
            /// \endcode
            Rotation m_q_A_B;
            /// The vector from the origin of A to the origin of B
            /// expressed in A
            Position m_A_t_A_B;
        };

        typedef QuatTransformationTemplate<double> QuatTransformation;

        template <typename Scalar>
        std::ostream &operator<<(std::ostream &out,
                                 const QuatTransformationTemplate<Scalar> &pose);

        // Exponential interpolation (i.e., Slerp) in SO(3) and linear interpolation in
        // R3. Lambda is in [0, 1], with 0 returning T_a, and 1 returning T_b.
        template <typename Scalar>
        inline QuatTransformationTemplate<Scalar> InterpolateComponentwise(
            const QuatTransformationTemplate<Scalar> &T_a,
            const QuatTransformationTemplate<Scalar> &T_b, const double lambda);
    } // namespace minimal
} // namespace kindr

#include <kindr/minimal/implementation/quat_transformation_realization.h>

#endif // KINDR_MINIMAL_QUAT_TRANSFORMATION_H_
