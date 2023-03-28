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

#include "mivins_api.h"
#include <svo/svo_interface.h>

std::shared_ptr<mivins::SvoInterface> svo_interface_;

int MiVins_StartUp(std::string config_file_, std::string calib_file_)
{
    svo_interface_ = std::make_shared<mivins::SvoInterface>(config_file_, calib_file_);
    return 1;
}

int MiVins_GetImuStatus()
{
    if(!svo_interface_)
    {
        std::cout<<"Please call MiVins_StartUp() first getimu!!"<< std::endl;
        return 0;
    }
    else
    {
        return svo_interface_->imuHandlerValid();
    }
    
}

int MiVins_ImuInput(const double ts, 
                const Eigen::Vector3d &acc_imu, 
                const Eigen::Vector3d &gyr_imu)
{
    if(!svo_interface_)
    {
        std::cout<<"Please call MiVins_StartUp() first imuinput!!"<< std::endl;
        return 0;
    }
    else
    {
        svo_interface_->inputImuData(ts, acc_imu, gyr_imu);
        return 1;
    }
}

int MiVins_ImgInput(const int64_t ts, 
  const std::vector<cv::Mat> &images, 
  const std::map<int, cv::Mat> &depths)
{
    if(!svo_interface_)
    {
        std::cout<<"Please call MiVins_StartUp() first imginput!!"<< std::endl;
        return 0;
    }
    else
    {
        svo_interface_->processImageData(ts, images, depths);
        return 1;
    }
}

int MiVins_GetOdomStatus()
{
    if(!svo_interface_)
    {
        std::cout<<"Please call MiVins_StartUp() first getodom!!"<< std::endl;
        return 0;
    }
    else
    {
        return svo_interface_->odomHandlerValid();
    }
}

int MiVins_OdomInput(const double ts, 
                const Eigen::Quaterniond &orientation, 
                const Eigen::Vector3d &position, 
                const Eigen::Vector3d &linear_velocity, 
                const Eigen::Vector3d &angular_velocity)
{
    if(!svo_interface_)
    {
        std::cout<<"Please call MiVins_StartUp() first odominput!!"<< std::endl;
        return 0;
    }
    else
    {
        svo_interface_->inputOdomData(ts, orientation, position, linear_velocity, angular_velocity);
        return 1;
    }
}

int MiVins_GetPipelineType()
{
    if(!svo_interface_)
    {
        std::cout<<"Please call MiVins_StartUp() first !!"<< std::endl;
        return 0;
    }
    else
    {
        return svo_interface_->getPipelineType();
    }
}

int MiVins_GetStage() 
{
    return svo_interface_->getStage();
}

std::vector<std::vector<double>> MiVins_GetLastFramesPose(const std::string sensor_type)
{
    return svo_interface_->GetLastFramesPose(sensor_type);
}

bool MiVins_GetAlignedOdom(const double timestamp, Eigen::Matrix4d &odom_pose)
{
    return svo_interface_->getAlignedOdom(timestamp, odom_pose);
}

bool MiVins_GetLatestAlignedOdom(double &timestamp, Eigen::Matrix4d &odom_pose)
{
    return svo_interface_->getLatestAlignedOdom(timestamp, odom_pose);
}

std::vector<std::vector<double>> MiVins_GetLastFramesCamPose()
{
    return svo_interface_->GetLastFramesCamPose();
}
std::vector<double> MiVins_GetLastFramesIMUPose() {
    return svo_interface_->GetLastFramesIMUPose();
}


int MiVins_ShutDown()
{
    if(!svo_interface_)
    {
        std::cout<<"Please call MiVins_StartUp() first ------------!!"<< std::endl;
        return 0;
    }
    else
    {
        svo_interface_->SvoShutdown();
        //svo_interface_.reset();//for finish map 
        std::cout<< "[MIVINS SHUTDOWN] mivins shutdown finish!" << std::endl;
        return 1;
    }
}

void MiVins_SaveFile()
{
    svo_interface_->savefile();
}

void MiVins_finish_SaveFile()
{
    svo_interface_->finishfile();
}

std::vector<double> MiVins_GetCameraImuCalib() {
    return svo_interface_->GetCameraImuCalib();
}

std::vector<double> MiVins_GetImuBaselinkCalib() {
    return svo_interface_->GetImuBaselinkCalib();
}

int MiVins_SetStableMode()
{
    if(!svo_interface_)
        return 0;
    else
        svo_interface_->stable_mode_first_stage_ = true;

    return 1;
}

int MiVins_StatusInput(const int motion_id)
{
    if(!svo_interface_)
        return 0;
    else
        svo_interface_->inputStatusData(motion_id);

    return 1;
}