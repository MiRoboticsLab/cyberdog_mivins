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

#include <svo/pose_update.h>
#include <mivins/utils/math_utils.h>
#include <mivins/transformation_optimizer.h>

namespace mivins
{

PoseUpdate::PoseUpdate(const PoseUpdateOptions &options):options_(options)
{
    R_.setIdentity();
    t_.setZero();
    delta_z_.setZero();
    stage_ = PoseUpdateStage::kStart;
    transformation_optimizer_.reset(new TransformationOptimizer(TransformationOptimizer::getDefaultSolverOptions()));

    // std::string path = getenv("HOME");
    // trace_pose_update_.open(path+"/savelog/pose_update.txt");
    // trace_pose_update_.precision(16);
}

PoseUpdate::~PoseUpdate()
{
    vio_pose_.clear();
    tar_pose_.clear();

    transformation_optimizer_ = nullptr;
}

void PoseUpdate::updatePose(Transformation &Twb_i, Transformation &Twb_o, bool is_kf)
{
    vio_pose_.push_front(Twb_i);
    tar_pose_.push_front(Twb_o);

    // if(!flag && stage_==PoseUpdateStage::kAdjusting)
    // {
    //     for(int i=0; i<tar_pose_.size(); ++i)
    //         trace_pose_update_ << tar_pose_[i].GetPosition().transpose() << std::endl;

    //     flag = true;
    // }

    if(stage_ == PoseUpdateStage::kAdjusting)
    {
        while(vio_pose_.size() > options_.adj_pose_num)
        {
            vio_pose_.pop_back();
            tar_pose_.pop_back();
        }
    }
    else
    {
        while(vio_pose_.size() > options_.ini_pose_num)
        {
            vio_pose_.pop_back();
            tar_pose_.pop_back();
        }
    }

    if(is_kf)
    {
        preProcessData();

        if(!isMoving())
            return;

        setUpdateStage();

        if(stage_ != PoseUpdateStage::kStart)
            computeTransformation();

        if(stage_ != PoseUpdateStage::kStart)
            if(options_.compensate_z)
                computeZDCompensation();
    }

    return;
}

PoseUpdateStage PoseUpdate::getStage()
{
    return stage_;
}

Eigen::Matrix3d PoseUpdate::getRot()
{
    return R_;
}

Eigen::Vector3d PoseUpdate::getTrl()
{
    return t_;
}

Eigen::Vector3d PoseUpdate::getCompZ()
{
    return delta_z_;
}

Transformation PoseUpdate::getTransformation()
{
    return Transformation(Quaternion(R_), t_);
}

Transformation PoseUpdate::getLastTarPose()
{
    Transformation last_tar;
    last_tar.setIdentity();

    if(!tar_pose_.empty())
        last_tar = tar_pose_.front();

    return last_tar;
}

void PoseUpdate::preProcessData()
{
    traveled_dis_.setZero();
    traveled_ang_ = 0.0;

    int data_num = vio_pose_.size();
    vio_pt_.resize(3, data_num);
    tar_pt_.resize(3, data_num);

    if(stage_ == PoseUpdateStage::kAdjusting)
    {
        for(int i=0; i<data_num; ++i)
        {
            vio_pt_.col(i) = vio_pose_[i].GetPosition();
            tar_pt_.col(i) = tar_pose_[i].GetPosition();
        }

        return;
    }

    double ang_max = 0.0;
    for(int i=0; i<data_num; ++i)
    {
        Eigen::Quaterniond tar_ori = tar_pose_[i].GetRotation().ToImplementation();
        Eigen::Quaterniond tar_ori_ini = tar_pose_[0].GetRotation().ToImplementation();

        Eigen::Quaterniond rot_diff = tar_ori_ini.inverse()*tar_ori;
        Eigen::AngleAxisd  ang_axis(rot_diff);
        if(ang_axis.angle() >= ang_max)
            ang_max = ang_axis.angle();

        vio_pt_.col(i) = vio_pose_[i].GetPosition();
        tar_pt_.col(i) = tar_pose_[i].GetPosition();
    }

    Eigen::Vector3d  tar_mean   = tar_pt_.rowwise().mean();
    Eigen::Matrix3Xd tar_demean = tar_pt_.colwise()-tar_mean;
    Eigen::Matrix3d  tar_cov    = tar_demean*tar_demean.transpose() / double(tar_pt_.cols()-1);

    Eigen::EigenSolver<Eigen::Matrix3d> eigenSolver(tar_cov);
    Eigen::Matrix3d eigenVectors = eigenSolver.pseudoEigenvectors();
    eigenVectors.col(2) = eigenVectors.col(0).cross(eigenVectors.col(1));
    eigenVectors.col(0) = eigenVectors.col(1).cross(eigenVectors.col(2));
    eigenVectors.col(1) = eigenVectors.col(2).cross(eigenVectors.col(0));

    Eigen::Quaterniond q(eigenVectors);
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();

    Eigen::Matrix3Xd tar_rotated = R.transpose()*tar_pt_;
    Eigen::Vector3d tar_min  = tar_rotated.rowwise().minCoeff();
    Eigen::Vector3d tar_max  = tar_rotated.rowwise().maxCoeff();
    Eigen::Vector3d traj_size = tar_max-tar_min;

    double tmp;
    if(traj_size(0) < traj_size(1)) 
    {
        tmp = traj_size(0);
        traj_size(0) = traj_size(1);
        traj_size(1) = tmp;
    }
    if(traj_size(1) < traj_size(2))
    {
        tmp = traj_size(1);
        traj_size(1) = traj_size(2);
        traj_size(2) = tmp;
    }
    if(traj_size(0) < traj_size(1)) 
    {
        tmp = traj_size(0);
        traj_size(0) = traj_size(1);
        traj_size(1) = tmp;
    }

    traveled_dis_ = traj_size;
    traveled_ang_ = ang_max;

    return;
}

void PoseUpdate::setUpdateStage()
{
    if(stage_ == PoseUpdateStage::kStart)
    {
        if(checkAdjustCond())
        {
            stage_ = PoseUpdateStage::kInitializing;
            return;
        }
        else if(checkInitiaCond())
        {
            stage_ = PoseUpdateStage::kInitializing;
            return;
        }
        else    
            return;
    }
    else if(stage_ == PoseUpdateStage::kInitializing)
    {
        if(checkAdjustCond())
        {
            stage_ = PoseUpdateStage::kAdjusting;
            return;
        }
        else
            return;
    }
    else
        return;

    return;
}


bool PoseUpdate::checkInitiaCond()
{
    if(traveled_dis_.norm()>options_.initia_th && vio_pose_.size()>options_.opt_pose_num)
        return true;

    return false;
}

bool PoseUpdate::checkAdjustCond()
{
    if(poseEnough())
    {
        if(traveled_dis_(1)>options_.adj_t1 || traveled_ang_>options_.adj_r1)
            return true;
    }
    else
    {
        if(traveled_dis_(1)>options_.adj_t2 && traveled_ang_>options_.adj_r1)
            return true;

        if(traveled_ang_>options_.adj_r2)
            return true;
    }

    return false;
}

bool PoseUpdate::poseEnough()
{
    if(stage_ == PoseUpdateStage::kAdjusting)
        return vio_pose_.size() >= options_.adj_pose_num;
    else
        return vio_pose_.size() >= options_.ini_pose_num;
}

bool PoseUpdate::isMoving()
{
    if(stage_ == PoseUpdateStage::kAdjusting)
        return true;

    if(traveled_dis_.norm() > options_.moving_th)
        return true;

    return false;
}

void PoseUpdate::computeTransformation()
{
    if(stage_ == PoseUpdateStage::kAdjusting)
    {
        if(!options_.adjust_pose)
            return;

        Eigen::Matrix4d T = Eigen::umeyama(vio_pt_, tar_pt_, false);
        Eigen::Matrix3d R = T.block<3,3>(0,0);
        Eigen::Vector3d t = T.block<3,1>(0,3);

        Transformation T_tar_vio(T);
        T_tar_vio.GetRotation().Normalize();
        // trace_pose_update_ << "before:\n"
        //                    << T << std::endl;
        double inlier_error, inlier_ratio;
        // transformation_optimizer_->SetTranslationPrior(t, options_.t_prior_lambda);
        transformation_optimizer_->run(vio_pose_, tar_pose_, T_tar_vio, options_.opt_outlier_th, inlier_error, inlier_ratio);
        // trace_pose_update_ << "after:\n"
        //                    << T_tar_vio.GetTransformationMatrix() << std::endl;
        // trace_pose_update_ << "error: "  << inlier_error << std::endl;
        // trace_pose_update_ << "inlier: " << inlier_ratio << std::endl;
        // trace_pose_update_ << "-----------------------------" << std::endl;

        if (inlier_ratio < options_.opt_inlier_ratio || 
            inlier_error > options_.opt_inlier_error)
            return;
        
        R = T_tar_vio.GetRotationMatrix();
        t = T_tar_vio.GetPosition();

        t(0) = t_(0); t(1) = t_(1);
        Eigen::Vector3d euler  = vk::dcm2rpy(R);
        Eigen::Vector3d euler_ = vk::dcm2rpy(R_);
        euler(2) = euler_(2);
        R = vk::rpy2dcm(euler);

        int used_cnt = 0;
        Eigen::Vector3d last_pos(0,0,0);
        Eigen::Vector3d curr_pos(0,0,0);
        for(int i=0; i<vio_pose_.size(); ++i)
        {
            if(used_cnt == 5)
                break;

            last_pos += R_*vio_pose_[i].GetPosition()+t_;
            curr_pos += R *vio_pose_[i].GetPosition()+t;
            ++used_cnt;
        }
        
        if(used_cnt == 0)
        {
            last_pos.setZero();
            curr_pos.setZero();
        }
        else
        {
            last_pos /= static_cast<double>(used_cnt);
            curr_pos /= static_cast<double>(used_cnt);
        }
        Eigen::Vector3d delta_t = curr_pos-last_pos;

        R_ = R;
        t_ = t-delta_t;
    }
    else
    {
        Eigen::Matrix4d T = Eigen::umeyama(vio_pt_, tar_pt_, false);
        Eigen::Matrix3d R = T.block<3,3>(0,0);
        Eigen::Vector3d t = T.block<3,1>(0,3);

        Transformation T_tar_vio(T);
        T_tar_vio.GetRotation().Normalize();
        // trace_pose_update_ << "before:\n"
        //                    << T << std::endl;
        double inlier_error, inlier_ratio;
        // transformation_optimizer_->SetTranslationPrior(t, options_.t_prior_lambda);
        transformation_optimizer_->run(vio_pose_, tar_pose_, T_tar_vio, options_.opt_outlier_th, inlier_error, inlier_ratio);
        // trace_pose_update_ << "after:\n"
        //                    << T_tar_vio.GetTransformationMatrix() << std::endl;
        // trace_pose_update_ << "error: "  << inlier_error << std::endl;
        // trace_pose_update_ << "inlier: " << inlier_ratio << std::endl;
        // trace_pose_update_ << "-----------------------------" << std::endl;

        if (inlier_ratio < options_.opt_inlier_ratio || 
            inlier_error > options_.opt_inlier_error || 
            vio_pose_.size() < options_.opt_pose_num)
        {
            if(!begin_init_)
                stage_ = PoseUpdateStage::kStart;
            return;
        }

        R = T_tar_vio.GetRotationMatrix();
        t = T_tar_vio.GetPosition();

        int used_cnt = 0;
        Eigen::Vector3d last_pos(0,0,0);
        Eigen::Vector3d curr_pos(0,0,0);
        if(!begin_init_)
        {
            for(int i=0; i<vio_pose_.size(); ++i)
            {
                if(used_cnt == 5)
                    break;

                last_pos += tar_pose_[i].GetPosition();
                curr_pos += R*vio_pose_[i].GetPosition()+t;
                ++used_cnt;
            }
            begin_init_ = true;
        }
        else
        {
            for(int i=0; i<vio_pose_.size(); ++i)
            {
                if(used_cnt == 5)
                    break;

                last_pos += R_*vio_pose_[i].GetPosition()+t_;
                curr_pos += R *vio_pose_[i].GetPosition()+t;
                ++used_cnt;
            }
        }
        
        if(used_cnt == 0)
        {
            last_pos.setZero();
            curr_pos.setZero();
        }
        else
        {
            last_pos /= static_cast<double>(used_cnt);
            curr_pos /= static_cast<double>(used_cnt);
        }
        Eigen::Vector3d delta_t = curr_pos-last_pos;

        R_ = R;
        t_ = t-delta_t;
    }

    return;
}

void PoseUpdate::computeZDCompensation()
{
    int used_cnt = 0;
    Eigen::Vector3d dst_p(0,0,0);
    Eigen::Vector3d tar_p(0,0,0);
    for(int i=0; i<vio_pose_.size(); ++i)
    {
        if(used_cnt == options_.adj_z_n)
            break;

        dst_p += R_*vio_pose_[i].GetPosition()+t_;
        tar_p += tar_pose_[i].GetPosition();
        ++used_cnt;
    }

    if(used_cnt == 0)
    {
        dst_p.setZero();
        tar_p.setZero();
    }
    else
    {
        dst_p /= static_cast<double>(used_cnt);
        tar_p /= static_cast<double>(used_cnt);
    }

    Eigen::Vector3d delta_p = tar_p-dst_p;
    delta_z_ = Eigen::Vector3d(0, 0, delta_p(2));
}

}