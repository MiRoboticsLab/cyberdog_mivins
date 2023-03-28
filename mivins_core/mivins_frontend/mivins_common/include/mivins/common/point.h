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

#pragma once

#include <array>
#include <atomic>
#include <iostream>
#include <unordered_map>

#include <glog/logging.h>
#include <mivins/common/types.h>

namespace mivins
{
    //*********************************************************************************//
    //                Thread-safe Global point-ID provider.                            //
    //*********************************************************************************//
    class GlobalPointId
    {
    public:
        GlobalPointId() = delete;
        static void SetPointId(int init_point_id)
        {
            s_i_point_id_ = init_point_id;
        }
        // first get, then add, thread safe
        static int GetNewPointId()
        {
            return s_i_point_id_.fetch_add(1);
        }

    private:
        static std::atomic<int> s_i_point_id_;
    };

    struct Point3dObservation
    {
        FrameWeakPtr frame;
        int frame_id;
        size_t keypoint_index;
        Point3dObservation(const FramePtr &_frame, const size_t _feature_index);

        inline bool operator==(const Point3dObservation &other) const
        {
            CHECK(frame.lock() && other.frame.lock());
            return frame.lock().get() == other.frame.lock().get() &&
                   frame_id == other.frame_id && keypoint_index == other.keypoint_index;
        }

        /// \brief Less than operator. Compares first multiframe ID, then camera index,
        ///        then keypoint index.
        bool operator<(const Point3dObservation &rhs) const
        {
            if (frame_id == rhs.frame_id)
            {
                return keypoint_index < rhs.keypoint_index;
            }
            return frame_id < rhs.frame_id;
        }
    };
    using Point3dObservationVec = std::vector<Point3dObservation>;
    using Matrix23d = Eigen::Matrix<double, 2, 3>;

    /// A 3D point on the surface of the scene.
    class Point
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // observation saves frame-id and pointer to feature
        typedef std::vector<Point3dObservation> Point3dObservationVec;

        enum PointType
        {
            TYPE_EDGELET_SEED,
            TYPE_CORNER_SEED,
            TYPE_EDGELET,
            TYPE_CORNER
        };

        int id_;                                  //!< Unique ID of the point.
        Position pos3d_in_w;                      //!< 3d pos of the point in the world coordinate frame.
        Point3dObservationVec obs_;               //!< References to keyframes which observe the point
        Eigen::Vector3d normal_;                  //!< Surface normal at point.
        Eigen::Matrix2d normal_information_;      //!< Inverse covariance matrix of normal estimation.
        bool normal_set_ = false;                 //!< Flag whether the surface normal was estimated or not.
        uint64_t last_published_ts_ = 0;          //!< Timestamp of last publishing.
        std::array<int, 8> last_projected_kf_id_; //!< Flag for the reprojection: don't reproject a pt twice in the same camera
        PointType type_ = TYPE_CORNER;            //!< Quality of the point.
        int n_failed_reproj_ = 0;                 //!< Number of failed reprojections. Used to assess the quality of the point.
        int n_succeeded_reproj_ = 0;              //!< Number of succeeded reprojections. Used to assess the quality of the point.
        int last_structure_optim_ = 0;            //!< Timestamp of last point optimization

        // bundle adjustment:
        bool in_ba_graph_ = false;    //!< Was this point already added to the iSAM bundle adjustment graph?
        int64_t last_ba_update_ = -1; //!< Timestamp of last estimate in bundle adjustment.
        static std::atomic_uint64_t global_map_value_version_;

        bool is_constant_ = false;
        /// Default constructor.
        Point(const Eigen::Vector3d &pos);

        /// Constructor with id: Only for testing!
        Point(const int id, const Eigen::Vector3d &pos);

        void SetPoint(const Eigen::Vector3d &pos);

        void SetConstant() { is_constant_ = true; }
        /// Destructor.
        ~Point();

        // no copy
        Point(const Point &) = delete;
        Point &operator=(const Point &) = delete;

        /// Add a reference to a frame.
        void AddObservation(const FramePtr &frame, const size_t feature_index);

        /// Remove observation via frame-ID.
        void RemoveObservation(int frame_id);

        /// Initialize point normal. The inital estimate will point towards the frame.
        void InitNormal();

        /// Get Frame with similar viewpoint.
        /// pos               -- input parameter
        /// ref_frame         -- output parameter
        /// ref_feature_index -- output parameter
        bool GetCloseViewObs(const Eigen::Vector3d &pos, FramePtr &ref_frame, size_t &ref_feature_index) const;

        /// Get parallax angle of triangulation. Useful to check if point is constrained.
        double GetTriangulationParallax() const;

        /// Get frame with seed of this point.
        FramePtr GetSeedFrame() const;

        /// Get number of observations.
        inline size_t NRefs() const { return obs_.size(); }

        /// Retriangulate the point from its observations.
        bool TriangulateLinear();

        /// update Hessian and Gradient of point based on one observation, using unit plane.
        void UpdateHessianGradientUnitPlane(
            const Eigen::Ref<BearingVector> &f,
            const Eigen::Vector3d &p_in_f,
            const Eigen::Matrix3d &R_f_w,
            Eigen::Matrix3d &A,
            Eigen::Vector3d &b,
            double &new_chi2);

        /// update Hessian and Gradient of point based on one observation, using unit sphere.
        void UpdateHessianGradientUnitSphere(
            const Eigen::Ref<BearingVector> &f,
            const Eigen::Vector3d &p_in_f,
            const Eigen::Matrix3d &R_f_w,
            Eigen::Matrix3d &A,
            Eigen::Vector3d &b,
            double &new_chi2);

        /// Optimize point position through minimizing the reprojection error.
        void Optimize(const size_t n_iter, bool using_bearing_vector = false);

        /// Print infos about the point.
        void Print(const std::string &s = "Point:") const;

        /// Return unique point identifier.
        inline int Id() const { return id_; }

        /// 3D position of point in world frame.
        inline const Eigen::Vector3d &pos() const { return pos3d_in_w; }

        /// Return type of point.
        inline const PointType &type() const { return type_; }

        /// Jacobian of point projection on unit plane (focal length = 1) in frame (f).
        inline static void Jacobian_xyz2uv(
            const Eigen::Vector3d &p_in_f,
            const Eigen::Matrix3d &R_f_w,
            mivins::Matrix23d &point_jac)
        {
            const double z_inv = 1.0 / p_in_f[2];
            const double z_inv_sq = z_inv * z_inv;
            point_jac(0, 0) = z_inv;
            point_jac(0, 1) = 0.0;
            point_jac(0, 2) = -p_in_f[0] * z_inv_sq;
            point_jac(1, 0) = 0.0;
            point_jac(1, 1) = z_inv;
            point_jac(1, 2) = -p_in_f[1] * z_inv_sq;
            point_jac = -point_jac * R_f_w;
        }

        /// Jacobian of point to unit bearing vector.
        inline static void Jacobian_xyz2f(
            const Eigen::Vector3d &p_in_f,
            const Eigen::Matrix3d &R_f_w,
            Eigen::Matrix3d &point_jac)
        {
            Eigen::Matrix3d J_normalize;
            double x2 = p_in_f[0] * p_in_f[0];
            double y2 = p_in_f[1] * p_in_f[1];
            double z2 = p_in_f[2] * p_in_f[2];
            double xy = p_in_f[0] * p_in_f[1];
            double yz = p_in_f[1] * p_in_f[2];
            double zx = p_in_f[2] * p_in_f[0];
            J_normalize << y2 + z2, -xy, -zx,
                -xy, x2 + z2, -yz,
                -zx, -yz, x2 + y2;
            J_normalize *= 1.0 / std::pow(x2 + y2 + z2, 1.5);
            point_jac = (-1.0) * J_normalize * R_f_w;
        }
    };

} // namespace mivins

namespace std
{

    inline ostream &operator<<(ostream &stream, const mivins::Point3dObservation &id)
    {
        stream << "(" << id.frame_id << ", " << id.keypoint_index << ")";
        return stream;
    }

} // namespace std
