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

#ifndef KINDR_MINIMAL_TRANSFORM_2D_H_
#define KINDR_MINIMAL_TRANSFORM_2D_H_

#include <ostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
//2d trans+rotate
namespace kindr
{
    namespace minimal
    {

        template <typename Scalar>
        using Position2DTemplate = Eigen::Matrix<Scalar, 2, 1>;

        template <typename Scalar>
        using Rotation2DTemplate = Eigen::Rotation2D<Scalar>;

        template <typename Scalar>
        class Transformation2DTemplate
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
            using Matrix2X = Eigen::Matrix<Scalar, 2, Eigen::Dynamic>;

            using Rotation = Rotation2DTemplate<Scalar>;
            using Position = Position2DTemplate<Scalar>;
            using RotationMatrix = Eigen::Matrix<Scalar, 2, 2>;
            using TransformationMatrix = Eigen::Matrix<Scalar, 3, 3>;

            Transformation2DTemplate();

            Transformation2DTemplate(const Rotation r_A_B, const Position &A_t_A_B);

            explicit Transformation2DTemplate(const TransformationMatrix &T);

            ~Transformation2DTemplate();

            void SetIdentity();

            // Non-const getter for the position vector.
            Position &GetPosition();

            // Const getter for the position vector.
            const Position &GetPosition() const;

            // Non-const getter for the rotation. Setting a new rotation angle can be done
            // as follows:
            //
            // Transformation2D T;
            // T.GetRotation().angle() = new_angle;
            Rotation &GetRotation();

            // Const getter for the rotation.
            const Rotation &GetRotation() const;

            RotationMatrix GetRotationMatrix() const;

            TransformationMatrix GetTransformationMatrix() const;

            // Get the rotation angle and the position as a vector: [angle, x, y]
            Eigen::Matrix<Scalar, 3, 1> AsVector() const;

            // Compose two transformations.
            Transformation2DTemplate<Scalar> operator*(
                const Transformation2DTemplate<Scalar> &rhs) const;

            // Transform a point.
            Vector2 operator*(const Vector2 &rhs) const;

            // Transform a point.
            Vector2 Transform(const Vector2 &rhs) const;

            // Transform points.
            Matrix2X TransformVectorized(const Matrix2X &rhs) const;

            // Returns a copy of the inverted transformation.
            Transformation2DTemplate<Scalar> Inverse() const;

            // Check binary equality.
            bool operator==(const Transformation2DTemplate<Scalar> &rhs) const;

            // Check binary inequality.
            bool operator!=(const Transformation2DTemplate<Scalar> &rhs) const;

            // Cast scalar elements to another type.
            template <typename ScalarAfterCast>
            Transformation2DTemplate<ScalarAfterCast> Cast() const;

        private:
            // The rotation that takes vectors from B to A.
            //
            // A_v = r_A_B * B_v;
            Rotation m_r_A_B;

            // The vector from the origin of A to the origin of B, expressed in A.
            Position m_A_t_A_B;
        };

        using Position2D = Position2DTemplate<double>;
        using Rotation2D = Rotation2DTemplate<double>;
        using Transformation2D = Transformation2DTemplate<double>;

        template <typename Scalar>
        std::ostream &operator<<(std::ostream &out,
                                 const Transformation2DTemplate<Scalar> &rhs);

    } // namespace minimal
} // namespace kindr

#include "kindr/minimal/implementation/transform_2d_realization.h"

#endif // KINDR_MINIMAL_TRANSFORM_2D_H_
