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

#include <mivins/common/point.h>

#include <mivins/utils/math_utils.h>
#include <mivins/common/logging.h>
#include <mivins/common/frame.h>

namespace mivins
{

    std::atomic<int> GlobalPointId::s_i_point_id_{0};

    Point3dObservation::Point3dObservation(const FramePtr &_frame, const size_t _feature_index)
        : frame(_frame), frame_id(_frame->GetFrameId()), keypoint_index(_feature_index)
    {
        ;
    }

    Point::Point(const Eigen::Vector3d &pos)
        : Point(GlobalPointId::GetNewPointId(), pos)
    {
        ;
    }

    void Point::SetPoint(const Eigen::Vector3d &pos)
    {
        pos3d_in_w = pos;
    }
    Point::Point(const int id, const Eigen::Vector3d &pos)
        : id_(id), pos3d_in_w(pos)
    {
        last_projected_kf_id_.fill(-1);
    }

    Point::~Point()
    {
    }

    std::atomic_uint64_t Point::global_map_value_version_{0u};

    void Point::AddObservation(const FramePtr &frame, const size_t feature_index)
    {
        CHECK_NOTNULL(frame.get());

        // check that we don't have yet a reference to this frame
        // TODO(cfo): maybe we should use a std::unordered_map to store the observations.
        const auto id = frame->GetFrameId();
        auto it = std::find_if(obs_.begin(), obs_.end(),
                               [&](const Point3dObservation &i)
                               { return i.frame_id == id; });
        if (it == obs_.end())
        {
            obs_.emplace_back(Point3dObservation(frame, feature_index));
        }
        else
        {
            CHECK_EQ(it->keypoint_index, feature_index);
        }
    }

    void Point::RemoveObservation(int frame_id)
    {
        obs_.erase(
            std::remove_if(obs_.begin(), obs_.end(),
                           [&](const Point3dObservation &o)
                           { return o.frame_id == frame_id; }),
            obs_.end());
    }

    void Point::InitNormal()
    {
        CHECK(!obs_.empty()) << "initializing normal without any observation";

        if (const FramePtr &frame = obs_.front().frame.lock())
        {
            BearingVector f = frame->f_vec_.col(obs_.front().keypoint_index);
            normal_ = frame->T_f_w_.GetRotation().InverseRotate(-f);
            normal_information_ = Eigen::Matrix2d::Identity(); //DiagonalMatrix<double,3,3>(pow(20/(pos3d_in_w-ftr->frame->GetCameraPosInWorld()).norm(),2), 1.0, 1.0);
            normal_set_ = true;
        }
        else
            LOG_ERROR_STREAM("could not unlock weak_ptr<frame> in normal initialization");
    }
    /// Get Frame with similar viewpoint.
    /// pos               -- input parameter
    /// ref_frame         -- output parameter
    /// ref_feature_index -- output parameter
    bool Point::GetCloseViewObs(
        const Eigen::Vector3d &framepos,
        FramePtr &ref_frame,
        size_t &ref_feature_index) const
    {
        double min_cos_angle = 0.0; // cos 0 = 90 degree
        Eigen::Vector3d obs_dir(framepos - pos3d_in_w);
        obs_dir.normalize();

        //
        //  TODO: For edgelets, find another view that reduces an epipolar line that
        //        is orthogonal to the gradient!
        //

        // TODO: get frame with same point of view AND same pyramid level!
        for (const Point3dObservation &obs : obs_)
        {
            if (FramePtr frame = obs.frame.lock())
            {
                Eigen::Vector3d dir(frame->GetCameraPosInWorld() - pos3d_in_w);
                dir.normalize();
                const double cos_angle = obs_dir.dot(dir);
                if (cos_angle > min_cos_angle)
                {
                    min_cos_angle = cos_angle;
                    ref_frame = frame;
                    ref_feature_index = obs.keypoint_index;
                }
            }
            else
            {
                LOG_DEBUG_STREAM("could not unlock weak_ptr<Frame> in Point::GetCloseViewObs"
                                 << ", Point-ID = " << id_
                                 << ", Point-nObs = " << obs_.size()
                                 << ", Frame-ID = " << obs.frame_id
                                 << ", Feature-ID = " << obs.keypoint_index
                                 << ", Point-Type = " << type_);
                return false;
            }
        }
        if (min_cos_angle < 0.4) // assume that observations larger than 66° are useless
        {
            LOG_DEBUG_STREAM("GetCloseViewObs(): obs is from too far away: " << min_cos_angle);
            return false;
        }
        return true;
    }
    /// iterate the obs to get the max parallax with the oldest observation
    double Point::GetTriangulationParallax() const
    {
        CHECK(!obs_.empty()) << "getTriangualtionParallax(): obs_ is empty!";

        const FramePtr ref_frame = obs_.front().frame.lock();
        if (!ref_frame)
        {
            LOG_ERROR_STREAM("getTriangualtionParallax(): Could not lock ref_frame");
            return 0.0;
        }

        const Eigen::Vector3d r = (ref_frame->GetCameraPosInWorld() - pos3d_in_w).normalized();
        double max_parallax = 0.0;
        for (const Point3dObservation &obs : obs_)
        {
            if (const FramePtr frame = obs.frame.lock())
            {
                const Eigen::Vector3d v = (frame->GetCameraPosInWorld() - pos3d_in_w).normalized();
                const double parallax = std::acos(r.dot(v));
                max_parallax = std::max(parallax, max_parallax);
            }
        }
        return max_parallax;
    }
    // get the oldest observation
    FramePtr Point::GetSeedFrame() const
    {
        if (obs_.empty())
            return nullptr;

        // the seed should be in the observation with the smallest id
        if (auto ref_frame = obs_.front().frame.lock())
            return ref_frame;
        else
            LOG_ERROR_STREAM("could not lock weak_ptr<Frame> in point");
        return nullptr;
    }

    bool Point::TriangulateLinear()
    {
        const size_t n_obs = obs_.size();
        if (n_obs < 2)
        {
            return false; // not enough measurements to triangulate.
        }
        Eigen::Matrix3Xd f_world; // bearing vectors in world coordinates
        Eigen::Matrix3Xd p_world; // position vectors of cameras
        f_world.resize(Eigen::NoChange, n_obs);
        p_world.resize(Eigen::NoChange, n_obs);
        size_t index = 0;
        for (const Point3dObservation &obs : obs_)
        {
            if (const FramePtr frame = obs.frame.lock())
            {
                const Transformation T_world_cam = frame->T_world_cam();
                f_world.col(index) = T_world_cam * frame->f_vec_.col(obs.keypoint_index);
                p_world.col(index) = T_world_cam.GetPosition();
                ++index;
            }
        }
        if (index != n_obs)
        {
            LOG_ERROR_STREAM("TriangulateLinear failed, could not lock all frames.");
            return false;
        }

        // from aslam: triangulation.cc
        const Eigen::MatrixXd BiD = f_world *
                                    f_world.colwise().squaredNorm().asDiagonal().inverse();
        const Eigen::Matrix3d AxtAx = n_obs * Eigen::Matrix3d::Identity() -
                                      BiD * f_world.transpose();
        const Eigen::Vector3d Axtbx = p_world.rowwise().sum() - BiD *
                                                                    f_world.cwiseProduct(p_world).colwise().sum().transpose();

        Eigen::ColPivHouseholderQR<Eigen::Matrix3d> qr = AxtAx.colPivHouseholderQr();
        static constexpr double kRankLossTolerance = 1e-5;
        qr.setThreshold(kRankLossTolerance);
        const size_t rank = qr.rank();
        if (rank < 3)
        {
            return false; // unobservable
        }
        pos3d_in_w = qr.solve(Axtbx);
        return true;
    }

    void Point::UpdateHessianGradientUnitPlane(
        const Eigen::Ref<BearingVector> &f,
        const Eigen::Vector3d &p_in_f,
        const Eigen::Matrix3d &R_f_w,
        Eigen::Matrix3d &A,
        Eigen::Vector3d &b,
        double &new_chi2)
    {
        mivins::Matrix23d J;
        Point::Jacobian_xyz2uv(p_in_f, R_f_w, J);
        const Eigen::Vector2d e(vk::project2(f) - vk::project2(p_in_f));
        A.noalias() += J.transpose() * J;
        b.noalias() -= J.transpose() * e;
        new_chi2 += e.squaredNorm();
    }

    void Point::UpdateHessianGradientUnitSphere(
        const Eigen::Ref<BearingVector> &f,
        const Eigen::Vector3d &p_in_f,
        const Eigen::Matrix3d &R_f_w,
        Eigen::Matrix3d &A,
        Eigen::Vector3d &b,
        double &new_chi2)
    {
        Eigen::Matrix3d J;
        Point::Jacobian_xyz2f(p_in_f, R_f_w, J);
        const Eigen::Vector3d e = f - p_in_f.normalized();
        A.noalias() += J.transpose() * J;
        b.noalias() -= J.transpose() * e;
        new_chi2 += e.squaredNorm();
    }

    void Point::Optimize(const size_t n_iter, bool using_bearing_vector)
    {
        Eigen::Vector3d old_point = pos3d_in_w;
        double chi2 = 0.0;
        Eigen::Matrix3d A;
        Eigen::Vector3d b;

        if (obs_.size() < 2)
        {
            LOG_ERROR_STREAM("optimizing point with less than two observations");
            return;
        }

        const double eps = 0.0000000001;
        for (size_t i = 0; i < n_iter; i++)
        {
            A.setZero();
            b.setZero();
            double new_chi2 = 0.0;

            // compute residuals
            for (const Point3dObservation &obs : obs_)
            {
                if (const FramePtr &frame = obs.frame.lock())
                {
                    if (using_bearing_vector)
                    {
                        UpdateHessianGradientUnitSphere(
                            frame->f_vec_.col(obs.keypoint_index), frame->T_f_w_ * pos3d_in_w,
                            frame->T_f_w_.GetRotation().GetRotationMatrix(),
                            A, b, new_chi2);
                    }
                    else
                    {
                        UpdateHessianGradientUnitPlane(
                            frame->f_vec_.col(obs.keypoint_index), frame->T_f_w_ * pos3d_in_w,
                            frame->T_f_w_.GetRotation().GetRotationMatrix(),
                            A, b, new_chi2);
                    }
                }
                else
                    LOG_ERROR_STREAM("could not unlock weak_ptr<Frame> in Point::optimize");
            }

            // solve linear system
            const Eigen::Vector3d dp(A.ldlt().solve(b));

            // check if error increased
            if ((i > 0 && new_chi2 > chi2) || (bool)std::isnan((double)dp[0]))
            {
#ifdef POINT_OPTIMIZER_DEBUG
                std::cout << "it " << i
                          << "\t FAILURE \t new_chi2 = " << new_chi2 << std::endl;
#endif
                pos3d_in_w = old_point; // roll-back
                break;
            }

            // update the model
            Eigen::Vector3d new_point = pos3d_in_w + dp;
            old_point = pos3d_in_w;
            pos3d_in_w = new_point;
            chi2 = new_chi2;
#ifdef POINT_OPTIMIZER_DEBUG
            cout << "it " << i
                 << "\t Success \t new_chi2 = " << new_chi2
                 << "\t norm(b) = " << vk::Norm_max(b)
                 << endl;
#endif

            // stop when converged
            if (vk::norm_max(dp) <= eps)
                break;
        }
#ifdef POINT_OPTIMIZER_DEBUG
        cout << endl;
#endif
    }

    void Point::Print(const std::string &s) const
    {
        std::cout << s << std::endl;
        std::cout << "  id = " << id_ << std::endl;
        std::cout << "  pos = [" << pos3d_in_w.transpose() << "]" << std::endl;
        std::cout << "  num obs = " << obs_.size() << std::endl;
    }

} // namespace mivins
