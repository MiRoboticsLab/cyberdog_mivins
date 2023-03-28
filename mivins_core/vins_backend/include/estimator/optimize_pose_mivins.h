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
 
#include <thread>
#include <mutex>
#include <unordered_map>
#include <queue>
// #include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


#include <mivins/channel_imu.h>
#include <mivins/channel_odom.h>
#include <mivins/common/frame.h>
#include <mivins/common/types.h>

#include <mivins/pose_optimizer.h>
#include <mivins/direct/feature_detector_utilities.h>


class OptimizePose : public mivins::MiniLeastSquaresSolver<6, mivins::Transformation, OptimizePose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using SigmaComputer = mivins::MedianSigmaComputer;
    using LossFunction = mivins::TukeyLossFunction;
    using SolverOptions = mivins::MiniLeastSquaresSolverOptions;
    typedef std::shared_ptr<OptimizePose> Ptr;
    typedef Eigen::Matrix<double,6,6> Matrix6d;
    typedef Eigen::Matrix<double,2,6> Matrix26d;
    typedef Eigen::Matrix<double,3,6> Matrix36d;
    typedef Eigen::Matrix<double,1,6> Matrix16d;
    typedef Eigen::Matrix<double,6,1> Vector6d;
    enum class ErrorType {kUnitPlane, kBearingVectorDiff, kImagePlane} ;

    struct Statistics {
        double reproj_error_after;
        double reproj_error_before;
        Statistics()
            : reproj_error_after(0.0)
            , reproj_error_before(0.0)
        {}
    } stats_;

    OptimizePose(SolverOptions solver_options);
    virtual ~OptimizePose() = default;

    static SolverOptions getDefaultSolverOptions();

    void run(const mivins::FrameBundle::Ptr& frame_bundle, 
                    const std::vector<std::unordered_map<size_t, Eigen::Vector3d>> &id_pts_maps);


    void setRotationPrior(const mivins::Quaternion& R_frame_world, double lambda);
    
    void setTransformPrior(const mivins::Transformation &T_cur_world_prior, double lambda);

    inline size_t IterCount() const {
        return m_iter;
    }

    inline void SetErrorType(ErrorType type) {
        err_type_ = type;
    }


//protected:
    mivins::FrameBundle::Ptr frame_bundle_;
    double prior_lambda_;
    SigmaComputer sigma_computer_;
    LossFunction loss_function_;
    double measurement_sigma_ = 1.0;
    ErrorType err_type_ = ErrorType::kUnitPlane;
    double focal_length_ = 1.0; /// focal length TODO: should be different for every camera
    // std::ofstream ofs_reproj_errors_;
    std::vector<std::unordered_map<size_t, Eigen::Vector3d>> id_pts_maps_;

    double EvaluateError(
            const mivins::Transformation& T_imu_world,
            HessianMatrix* H,
            GradientVector* g);

    double evaluateErrorImpl(
            const mivins::Transformation& T_imu_world,
            HessianMatrix* H,
            GradientVector* g,
            std::vector<float>* unwhitened_errors);

    void Update(
            const State& T_frameold_from_world,
            const UpdateVector& dx,
            State& T_framenew_from_world);

    virtual void ApplyPrior(const State& current_model);
};

