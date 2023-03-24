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

#ifndef KINDR_MIN_ROTATION_QUATERNION_INL_H_
#define KINDR_MIN_ROTATION_QUATERNION_INL_H_

#include <glog/logging.h>
#include <kindr/minimal/rotation_quaternion.h>
#include <kindr/minimal/angle_axis.h>
//旋转变换
namespace kindr
{
    namespace minimal
    {

        template <typename Scalar>
        struct EPS
        {
            static constexpr Scalar Value() { return static_cast<Scalar>(1.0e-5); }
            static constexpr Scalar Normalization_value()
            {
                return static_cast<Scalar>(1.0e-4);
            }
        };
        template <>
        struct EPS<double>
        {
            static constexpr double Value() { return 1.0e-8; }
            static constexpr double Normalization_value() { return 1.0e-4; }
        };
        template <>
        struct EPS<float>
        {
            static constexpr float Value() { return 1.0e-5f; }
            static constexpr float Normalization_value() { return 1.0e-4f; }
        };

        /// \brief initialize to identity
        template <typename Scalar>
        RotationQuaternionTemplate<Scalar>::RotationQuaternionTemplate()
            : m_q_A_B(Implementation::Identity()) {}

        /// \brief initialize from real and imaginary components (real first)
        template <typename Scalar>
        RotationQuaternionTemplate<Scalar>::RotationQuaternionTemplate(
            Scalar w, Scalar x, Scalar y, Scalar z) : m_q_A_B(w, x, y, z)
        {
            CHECK_NEAR(SquaredNorm(), static_cast<Scalar>(1.0),
                       EPS<Scalar>::Normalization_value());
        }

        /// \brief initialize from real and imaginary components
        template <typename Scalar>
        RotationQuaternionTemplate<Scalar>::RotationQuaternionTemplate(
            Scalar real,
            const typename RotationQuaternionTemplate<Scalar>::Vector3 &imaginary) : m_q_A_B(real, imaginary[0], imaginary[1], imaginary[2])
        {
            CHECK_NEAR(SquaredNorm(), static_cast<Scalar>(1.0),
                       EPS<Scalar>::Normalization_value());
        }

        /// \brief initialize from an Eigen quaternion
        template <typename Scalar>
        RotationQuaternionTemplate<Scalar>::RotationQuaternionTemplate(
            const Implementation &quaternion) : m_q_A_B(quaternion)
        {
            CHECK_NEAR(SquaredNorm(), static_cast<Scalar>(1.0),
                       EPS<Scalar>::Normalization_value());
        }

        namespace detail
        {

            template <typename Scalar_>
            inline bool IsLessThenEpsilons4thRoot(Scalar_ x)
            {
                static const Scalar_ epsilon4thRoot = pow(std::numeric_limits<Scalar_>::epsilon(), 1.0 / 4.0);
                return x < epsilon4thRoot;
            }

            template <typename Scalar_>
            inline Scalar_ ArcSinXOverX(Scalar_ x)
            {
                if (IsLessThenEpsilons4thRoot(fabs(x)))
                {
                    return Scalar_(1.0) + x * x * Scalar_(1.0 / 6.0);
                }
                return asin(x) / x;
            }
        } // namespace detail

        /// \brief initialize from axis-scaled angle vector
        template <typename Scalar>
        RotationQuaternionTemplate<Scalar>::RotationQuaternionTemplate(
            const Vector3 &axis_scaled_angle)
        {
            *this = Exp(axis_scaled_angle);
        }

        /// \brief initialize from a rotation matrix
        template <typename Scalar>
        RotationQuaternionTemplate<Scalar>::RotationQuaternionTemplate(
            const RotationMatrix &matrix) : m_q_A_B(matrix)
        {
            CHECK(IsValidRotationMatrix(matrix)) << matrix;
        }

        template <typename Scalar>
        RotationQuaternionTemplate<Scalar>
        RotationQuaternionTemplate<Scalar>::FromApproximateRotationMatrix(
            const RotationMatrix &matrix)
        {
            // We still want the input matrix to resemble a rotation matrix to avoid
            // bug hiding.
            CHECK(IsValidRotationMatrix(
                matrix, static_cast<Scalar>(EPS<float>::Normalization_value())));
            // http://people.csail.mit.edu/bkph/articles/Nearest_Orthonormal_Matrix.pdf
            // as discussed in https://github.com/ethz-asl/kindr/issues/55 ,
            // code by Philipp Krüsi.
            Eigen::JacobiSVD<RotationMatrix> svd(matrix, Eigen::ComputeFullV);

            RotationMatrix correction =
                svd.matrixV().col(0) * svd.matrixV().col(0).transpose() /
                    svd.singularValues()(0) +
                svd.matrixV().col(1) * svd.matrixV().col(1).transpose() /
                    svd.singularValues()(1) +
                svd.matrixV().col(2) * svd.matrixV().col(2).transpose() /
                    svd.singularValues()(2);

            return RotationQuaternionTemplate<Scalar>(
                RotationMatrix(matrix * correction));
        }

        template <typename Scalar>
        RotationQuaternionTemplate<Scalar>::RotationQuaternionTemplate(
            const AngleAxisTemplate<Scalar> &angleAxis) : m_q_A_B(angleAxis.ToImplementation())
        {
        }

        template <typename Scalar>
        RotationQuaternionTemplate<Scalar>::~RotationQuaternionTemplate()
        {
        }

        /// \brief the real component of the quaternion
        template <typename Scalar>
        Scalar RotationQuaternionTemplate<Scalar>::w() const
        {
            return m_q_A_B.w();
        }

        /// \brief the first imaginary component of the quaternion
        template <typename Scalar>
        Scalar RotationQuaternionTemplate<Scalar>::x() const
        {
            return m_q_A_B.x();
        }

        /// \brief the second imaginary component of the quaternion
        template <typename Scalar>
        Scalar RotationQuaternionTemplate<Scalar>::y() const
        {
            return m_q_A_B.y();
        }

        /// \brief the third imaginary component of the quaternion
        template <typename Scalar>
        Scalar RotationQuaternionTemplate<Scalar>::z() const
        {
            return m_q_A_B.z();
        }

        /// \brief assignment operator
        template <typename Scalar>
        RotationQuaternionTemplate<Scalar> &
        RotationQuaternionTemplate<Scalar>::operator=(
            const RotationQuaternionTemplate<Scalar> &rhs)
        {
            if (this != &rhs)
            {
                m_q_A_B = rhs.m_q_A_B;
            }
            return *this;
        }

        /// \brief the imaginary components of the quaterion.
        template <typename Scalar>
        typename RotationQuaternionTemplate<Scalar>::Imaginary
        RotationQuaternionTemplate<Scalar>::ImaginaryComponents() const
        {
            return Imaginary(m_q_A_B.x(), m_q_A_B.y(), m_q_A_B.z());
        }

        /// \brief get the components of the quaternion as a vector (real first)
        template <typename Scalar>
        typename RotationQuaternionTemplate<Scalar>::Vector4
        RotationQuaternionTemplate<Scalar>::Vector() const
        {
            return Vector4(m_q_A_B.w(), m_q_A_B.x(), m_q_A_B.y(), m_q_A_B.z());
        }

        /// \brief set the quaternion by its values (real, imaginary)
        template <typename Scalar>
        void RotationQuaternionTemplate<Scalar>::SetValues(Scalar w, Scalar x,
                                                           Scalar y, Scalar z)
        {
            m_q_A_B = Implementation(w, x, y, z);
            CHECK_NEAR(SquaredNorm(), static_cast<Scalar>(1.0),
                       static_cast<Scalar>(1e-4));
        }

        /// \brief set the quaternion by its real and imaginary parts
        template <typename Scalar>
        void RotationQuaternionTemplate<Scalar>::SetParts(Scalar real,
                                                          const Imaginary &imag)
        {
            m_q_A_B = Implementation(real, imag[0], imag[1], imag[2]);
            CHECK_NEAR(SquaredNorm(), static_cast<Scalar>(1.0),
                       static_cast<Scalar>(1e-4));
        }

        /// \brief get a copy of the representation that is unique
        template <typename Scalar>
        RotationQuaternionTemplate<Scalar>
        RotationQuaternionTemplate<Scalar>::GetUnique() const
        {
            if (this->w() > 0)
            {
                return *this;
            }
            else if (this->w() < 0)
            {
                return RotationQuaternionTemplate<Scalar>(
                    -this->w(), -this->x(), -this->y(), -this->z());
            }
            // w == 0
            if (this->x() > 0)
            {
                return *this;
            }
            else if (this->x() < 0)
            {
                return RotationQuaternionTemplate<Scalar>(
                    -this->w(), -this->x(), -this->y(), -this->z());
            }
            // x == 0
            if (this->y() > 0)
            {
                return *this;
            }
            else if (this->y() < 0)
            {
                return RotationQuaternionTemplate<Scalar>(
                    -this->w(), -this->x(), -this->y(), -this->z());
            }
            // y == 0
            if (this->z() > 0)
            { // z must be either -1 or 1 in this case
                return *this;
            }
            else
            {
                return RotationQuaternionTemplate<Scalar>(
                    -this->w(), -this->x(), -this->y(), -this->z());
            }
        }

        /// \brief set the quaternion to its unique representation
        template <typename Scalar>
        RotationQuaternionTemplate<Scalar> &
        RotationQuaternionTemplate<Scalar>::SetUnique()
        {
            *this = GetUnique();
            return *this;
        }

        /// \brief set the quaternion to identity
        template <typename Scalar>
        RotationQuaternionTemplate<Scalar> &
        RotationQuaternionTemplate<Scalar>::SetIdentity()
        {
            m_q_A_B.setIdentity();
            return *this;
        }

        /// \brief set to random rotation
        template <typename Scalar>
        RotationQuaternionTemplate<Scalar> &
        RotationQuaternionTemplate<Scalar>::SetRandom()
        {
            Vector4 coeffs;
            coeffs.SetRandom().normalize();
            m_q_A_B = Implementation(coeffs(0), coeffs(1), coeffs(2), coeffs(3));
            this->setUnique();
            return *this;
        }

        /// \brief set to random rotation  with a given angle
        template <typename Scalar>
        RotationQuaternionTemplate<Scalar> &
        RotationQuaternionTemplate<Scalar>::SetRandom(Scalar angle_rad)
        {
            Vector3 rotation_axis;
            rotation_axis.SetRandom().normalize();
            m_q_A_B = Implementation(Eigen::AngleAxis<Scalar>(angle_rad, rotation_axis));
            return *this;
        }

        /// \brief get a copy of the quaternion inverted.
        template <typename Scalar>
        RotationQuaternionTemplate<Scalar>
        RotationQuaternionTemplate<Scalar>::Inverse() const
        {
            return Conjugated();
        }

        /// \brief get a copy of the quaternion inverted.
        template <typename Scalar>
        RotationQuaternionTemplate<Scalar>
        RotationQuaternionTemplate<Scalar>::Inverted() const
        {
            return Inverse();
        }

        /// \brief get a copy of the conjugate of the quaternion.
        template <typename Scalar>
        RotationQuaternionTemplate<Scalar>
        RotationQuaternionTemplate<Scalar>::Conjugated() const
        {
            // Own implementation since Eigen::conjugate does not use the correct
            // scalar type for the greater than zero comparison.
            return RotationQuaternionTemplate(
                Implementation(m_q_A_B.w(), -m_q_A_B.x(), -m_q_A_B.y(), -m_q_A_B.z()));
        }

        /// \brief rotate a vector, v
        template <typename Scalar>
        typename RotationQuaternionTemplate<Scalar>::Vector3
        RotationQuaternionTemplate<Scalar>::Rotate(
            const typename RotationQuaternionTemplate<Scalar>::Vector3 &v) const
        {
            return m_q_A_B * v;
        }

        /// \brief rotate vectors, v
        template <typename Scalar>
        typename RotationQuaternionTemplate<Scalar>::Matrix3X
        RotationQuaternionTemplate<Scalar>::RotateVectorized(
            const typename RotationQuaternionTemplate<Scalar>::Matrix3X &v) const
        {
            CHECK_GT(v.cols(), 0);
            return m_q_A_B.toRotationMatrix() * v;
        }

        /// \brief rotate a vector, v
        template <typename Scalar>
        typename RotationQuaternionTemplate<Scalar>::Vector4
        RotationQuaternionTemplate<Scalar>::Rotate4(
            const typename RotationQuaternionTemplate<Scalar>::Vector4 &v) const
        {
            typename RotationQuaternionTemplate<Scalar>::Vector4 vprime;
            vprime[3] = v[3];
            vprime.template head<3>() = m_q_A_B * v.template head<3>();
            return vprime;
        }

        /// \brief rotate a vector, v
        template <typename Scalar>
        typename RotationQuaternionTemplate<Scalar>::Vector3
        RotationQuaternionTemplate<Scalar>::InverseRotate(
            const typename RotationQuaternionTemplate<Scalar>::Vector3 &v) const
        {
            return m_q_A_B.inverse() * v;
        }

        /// \brief rotate a vector, v
        template <typename Scalar>
        typename RotationQuaternionTemplate<Scalar>::Vector4
        RotationQuaternionTemplate<Scalar>::InverseRotate4(
            const typename RotationQuaternionTemplate<Scalar>::Vector4 &v) const
        {
            typename RotationQuaternionTemplate<Scalar>::Vector4 vprime;
            vprime[3] = v[3];
            vprime.template head<3>() = m_q_A_B.inverse() * v.template head<3>();
            return vprime;
        }

        /// \brief cast to the implementation type
        template <typename Scalar>
        typename RotationQuaternionTemplate<Scalar>::Implementation &
        RotationQuaternionTemplate<Scalar>::ToImplementation()
        {
            return m_q_A_B;
        }

        /// \brief cast to the implementation type
        template <typename Scalar>
        const typename RotationQuaternionTemplate<Scalar>::Implementation &
        RotationQuaternionTemplate<Scalar>::ToImplementation() const
        {
            return m_q_A_B;
        }

        /// \brief get the norm of the quaternion
        template <typename Scalar>
        Scalar RotationQuaternionTemplate<Scalar>::Norm() const
        {
            return m_q_A_B.norm();
        }

        /// \brief get the squared norm of the quaternion
        template <typename Scalar>
        Scalar RotationQuaternionTemplate<Scalar>::SquaredNorm() const
        {
            return m_q_A_B.squaredNorm();
        }

        /// \brief enforce the unit length constraint
        template <typename Scalar>
        RotationQuaternionTemplate<Scalar> &
        RotationQuaternionTemplate<Scalar>::Normalize()
        {
            m_q_A_B.normalize();
            return *this;
        }

        template <typename Scalar>
        RotationQuaternionTemplate<Scalar>
        RotationQuaternionTemplate<Scalar>::operator*(
            const RotationQuaternionTemplate<Scalar> &rhs) const
        {
            CHECK(!std::is_arithmetic<Scalar>::value) << "Please provide a specialized "
                                                         "function for this specific arithmetic type. This function is only a "
                                                         "workaround for non-arithmetic types.";
            Implementation result = m_q_A_B * rhs.m_q_A_B;

            // Check if the multiplication has resulted in the quaternion no longer being
            // approximately normalized.
            // Cover the case of non-arithmetic types that may not provide an
            // implementation of std::abs.
            Scalar signed_norm_diff = result.squaredNorm() - static_cast<Scalar>(1.0);
            if ((signed_norm_diff > EPS<Scalar>::Normalization_value()) ||
                (signed_norm_diff < -EPS<Scalar>::Normalization_value()))
            {
                // renormalize
                result.Normalize();
            }
            return RotationQuaternionTemplate<Scalar>(result);
        }

        template <>
        inline RotationQuaternionTemplate<float>
        RotationQuaternionTemplate<float>::operator*(
            const RotationQuaternionTemplate<float> &rhs) const
        {
            Implementation result = m_q_A_B * rhs.m_q_A_B;
            NormalizationHelper(&result);
            return RotationQuaternionTemplate<float>(result);
        }

        template <>
        inline RotationQuaternionTemplate<double>
        RotationQuaternionTemplate<double>::operator*(
            const RotationQuaternionTemplate<double> &rhs) const
        {
            Implementation result = m_q_A_B * rhs.m_q_A_B;
            NormalizationHelper(&result);
            return RotationQuaternionTemplate<double>(result);
        }

        template <typename Scalar>
        RotationQuaternionTemplate<Scalar>
        RotationQuaternionTemplate<Scalar>::operator*(
            const AngleAxisTemplate<Scalar> &rhs) const
        {
            return *this * RotationQuaternionTemplate<Scalar>(rhs);
        }

        template <typename Scalar>
        std::ostream &operator<<(std::ostream &out,
                                 const RotationQuaternionTemplate<Scalar> &rhs)
        {
            out << rhs.vector();
            return out;
        }

        template <typename Scalar>
        typename RotationQuaternionTemplate<Scalar>::RotationMatrix
        RotationQuaternionTemplate<Scalar>::GetRotationMatrix() const
        {
            return m_q_A_B.matrix();
        }

        template <typename Scalar>
        Scalar RotationQuaternionTemplate<Scalar>::GetDisparityAngle(
            const RotationQuaternionTemplate<Scalar> &rhs) const
        {
            return AngleAxisTemplate<Scalar>(rhs * this->inverse()).GetUnique().angle();
        }

        template <typename Scalar>
        Scalar RotationQuaternionTemplate<Scalar>::GetDisparityAngle(
            const AngleAxisTemplate<Scalar> &rhs) const
        {
            return AngleAxisTemplate<Scalar>(rhs * this->inverse()).getUnique().angle();
        }

        template <typename Scalar>
        typename RotationQuaternionTemplate<Scalar>::Vector3
        RotationQuaternionTemplate<Scalar>::Log(const RotationQuaternionTemplate<Scalar> &q)
        {
            const Eigen::Matrix<Scalar, 3, 1> a = q.ImaginaryComponents();
            const Scalar na = a.norm();
            const Scalar eta = q.w();
            Scalar scale;
            if (fabs(eta) < na)
            { // use eta because it is more precise than na to calculate the scale. No singularities here.
                // check sign of eta so that we can be sure that log(-q) = log(q)
                if (eta >= 0)
                {
                    scale = acos(eta) / na;
                }
                else
                {
                    scale = -acos(-eta) / na;
                }
            }
            else
            {
                /*
     * In this case more precision is in na than in eta so lets use na only to calculate the scale:
     *
     * assume first eta > 0 and 1 > na > 0.
     *               u = asin (na) / na  (this implies u in [1, pi/2], because na i in [0, 1]
     *    sin (u * na) = na
     *  sin^2 (u * na) = na^2
     *  cos^2 (u * na) = 1 - na^2
     *                              (1 = ||q|| = eta^2 + na^2)
     *    cos^2 (u * na) = eta^2
     *                              (eta > 0,  u * na = asin(na) in [0, pi/2] => cos(u * na) >= 0 )
     *      cos (u * na) = eta
     *                              (u * na in [ 0, pi/2] )
     *                 u = acos (eta) / na
     *
     * So the for eta > 0 it is acos(eta) / na == asin(na) / na.
     * From some geometric considerations (mirror the setting at the hyper plane q==0) it follows for eta < 0 that (pi - asin(na)) / na = acos(eta) / na.
     */
                if (eta > 0)
                {
                    // For asin(na)/ na the singularity na == 0 can be removed. We can ask (e.g. Wolfram alpha) for its series expansion at na = 0. And that is done in the following function.
                    scale = detail::ArcSinXOverX(na);
                }
                else
                {
                    // the negative is here so that log(-q) == log(q)
                    scale = -detail::ArcSinXOverX(na);
                }
            }
            return a * (Scalar(2.0) * scale);
        }

        template <typename Scalar>
        RotationQuaternionTemplate<Scalar>
        RotationQuaternionTemplate<Scalar>::Exp(const Vector3 &dx)
        {
            // Method of implementing this function that is accurate to numerical precision from
            // Grassia, F. S. (1998). Practical parameterization of rotations using the exponential map. journal of graphics, gpu, and game tools, 3(3):29–48.
            double theta = dx.norm();
            // na is 1/theta sin(theta/2)
            double na;
            if (detail::IsLessThenEpsilons4thRoot(theta))
            {
                static const double one_over_48 = 1.0 / 48.0;
                na = 0.5 + (theta * theta) * one_over_48;
            }
            else
            {
                na = sin(theta * 0.5) / theta;
            }
            double ct = cos(theta * 0.5);
            return RotationQuaternionTemplate<Scalar>(ct,
                                                      dx[0] * na,
                                                      dx[1] * na,
                                                      dx[2] * na);
        }

        template <typename Scalar>
        typename RotationQuaternionTemplate<Scalar>::Vector3
        RotationQuaternionTemplate<Scalar>::Log() const
        {
            return Log(*this);
        }

        template <typename Scalar>
        bool RotationQuaternionTemplate<Scalar>::IsValidRotationMatrix(
            const RotationMatrix &matrix)
        {
            return IsValidRotationMatrix(matrix, EPS<Scalar>::Value());
        }

        template <typename Scalar>
        bool RotationQuaternionTemplate<Scalar>::IsValidRotationMatrix(
            const RotationMatrix &matrix, const Scalar threshold)
        {
            if (std::fabs(matrix.determinant() - static_cast<Scalar>(1.0)) > threshold)
            {
                VLOG(200) << matrix.determinant();
                VLOG(200) << matrix.determinant() - static_cast<Scalar>(1.0);
                return false;
            }
            if ((matrix * matrix.transpose() -
                 RotationMatrix::Identity())
                    .cwiseAbs()
                    .maxCoeff() > threshold)
            {
                VLOG(200) << matrix * matrix.transpose();
                VLOG(200) << matrix * matrix.transpose() - RotationMatrix::Identity();
                return false;
            }
            return true;
        }

        template <typename Scalar>
        template <typename ScalarAfterCast>
        RotationQuaternionTemplate<ScalarAfterCast>
        RotationQuaternionTemplate<Scalar>::Cast() const
        {
            // renormalization needed to allow casting to increased precision
            return RotationQuaternionTemplate<ScalarAfterCast>::ConstructAndRenormalize(
                GetRotationMatrix().template Cast<ScalarAfterCast>());
        }

        template <typename Scalar>
        void RotationQuaternionTemplate<Scalar>::NormalizationHelper(Implementation *quaternion) const
        {
            CHECK_NOTNULL(quaternion);
            // check if the multiplication has resulted in the quaternion no longer being
            // approximately normalized.
            if (std::abs(quaternion->squaredNorm() - static_cast<Scalar>(1)) >
                EPS<Scalar>::Normalization_value())
            {
                // renormalize
                quaternion->normalize();
            }
        }

    } // namespace minimal
} // namespace kindr
#endif // KINDR_MIN_ROTATION_QUATERNION_INL_H_
