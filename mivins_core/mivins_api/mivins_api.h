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

#ifndef __MIVINS_API_H__
#define __MIVINS_API_H__
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unordered_map>
#include <opencv2/opencv.hpp>


int MiVins_StartUp(std::string config_file_, std::string calib_file_);

int MiVins_GetImuStatus();

int MiVins_ImuInput(const double ts, const Eigen::Vector3d &acc_imu, const Eigen::Vector3d &gyr_imu);

int MiVins_ImgInput(
	const int64_t ts, 
	const std::vector<cv::Mat> &images, 
	const std::map<int, cv::Mat> &depths);

int MiVins_GetOdomStatus();

int MiVins_OdomInput(
	const double ts, 
	const Eigen::Quaterniond &orientation, 
	const Eigen::Vector3d &position, 
	const Eigen::Vector3d &vel_odom, 
	const Eigen::Vector3d &gyr_odom);
                     
bool MiVins_GetAlignedOdom(const double timestamp, Eigen::Matrix4d &odom_pose);

bool MiVins_GetLatestAlignedOdom(double &timestamp, Eigen::Matrix4d &odom_pose);

int MiVins_GetPipelineType();

int MiVins_ShutDown();

void MiVins_SaveFile();

void MiVins_finish_SaveFile();

int MiVins_GetStage();

std::vector<std::vector<double>> MiVins_GetLastFramesPose(const std::string sensor_type = "imu");

std::vector<std::vector<double>> MiVins_GetLastFramesCamPose();

std::vector<double> MiVins_GetCameraImuCalib();

std::vector<double> MiVins_GetLastFramesIMUPose();

std::vector<double> MiVins_GetImuBaselinkCalib();

int MiVins_SetStableMode();
int MiVins_StatusInput(const int motion_id);

#endif
