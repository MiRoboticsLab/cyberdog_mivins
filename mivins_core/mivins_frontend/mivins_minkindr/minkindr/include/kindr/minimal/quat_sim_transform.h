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

#ifndef KINDR_MINIMAL_QUAT_SIM_TRANSFORM_H_
#define KINDR_MINIMAL_QUAT_SIM_TRANSFORM_H_

#include <ostream>

#include <Eigen/Dense>

#include "kindr/minimal/quat_transformation.h"
//相似变换(no use)
namespace kindr
{
    namespace minimal
    {

        // Scale & rotate then translate = scale then transform.
        // In particular, the transformation matrix is:
        //
        //   R*s t     R t   s*I 0
        //   0   1  =  0 1 * 0   1
        template <typename Scalar>
        class QuatSimTransformTemplate
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            typedef QuatTransformationTemplate<Scalar> Transform;
            typedef QuatSimTransformTemplate<Scalar> Sim3;
            typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
            typedef Eigen::Matrix<Scalar, 7, 1> Vector7;
            typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Matrix3X;

            // Creates identity similarity transform.
            QuatSimTransformTemplate();

            QuatSimTransformTemplate(const Transform &T_A_B, const Scalar scale_A_B);

            QuatSimTransformTemplate(const Vector7 &log_vector);

            inline Vector3 operator*(const Vector3 &rhs) const;

            // Vectorized, applies operator * to each column vector.
            inline Matrix3X operator*(const Matrix3X &rhs) const;

            inline Sim3 operator*(const Sim3 &rhs) const;

            inline Sim3 Operator *(const Transform &rhs) const;

            inline Sim3 Inverse() const;

            inline Vector7 Log() const;

            inline Eigen::Matrix<Scalar, 4, 4> GetTransformationMatrix() const;
            inline const Transform &GetTransform() const { return T_A_B_; }
            inline Scalar GetScale() const { return scale_A_B_; }

            inline void SetScale(const Scalar scale_A_B) { scale_A_B_ = scale_A_B; }

        private:
            Transform T_A_B_;
            Scalar scale_A_B_;

            template <typename FriendScalar>
            friend std::ostream &operator<<(
                std::ostream &, const QuatSimTransformTemplate<FriendScalar> &);
        };

        typedef QuatSimTransformTemplate<double> QuatSimTransform;

        template <typename Scalar>
        inline QuatSimTransformTemplate<Scalar> operator*(
            const QuatTransformationTemplate<Scalar> &lhs,
            const QuatSimTransformTemplate<Scalar> &rhs);

        template <typename Scalar>
        std::ostream &operator<<(std::ostream &out,
                                 const QuatSimTransformTemplate<Scalar> &sim_3);

    } // namespace minimal
} // namespace kindr

#include "kindr/minimal/implementation/quat_sim_transform_realization.h"

#endif // KINDR_MINIMAL_QUAT_SIM_TRANSFORM_H_
