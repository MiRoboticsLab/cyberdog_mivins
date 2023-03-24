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

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>


#include <mivins/channel_imu.h>
#include <mivins/channel_odom.h>
#include <mivins/common/frame.h>
#include <mivins/common/types.h>
#include <mivins/common/camera.h>

#include <mivins/common/transformation.h>

// for optimize when update
#include <mivins/pose_optimizer.h>
#include <mivins/direct/feature_detector_utilities.h>

#include <mivins/bundle_adjustment.h>

namespace backend
{

typedef mivins::Quaternion       Quaternion;
typedef mivins::Transformation   Transformation; 

typedef mivins::Point            Point;
typedef mivins::PointPtr         PointPtr;
typedef mivins::BundleId         BundleId;
typedef mivins::Position         Position;
typedef mivins::Keypoint         Keypoint;
typedef mivins::Keypoints        Keypoints;
typedef mivins::BearingVector    BearingVector;
typedef mivins::GradientVector   GradientVector;

typedef mivins::Frame            Frame; 
typedef mivins::FramePtr         FramePtr; 
typedef mivins::FrameBundle      FrameBundle;
typedef mivins::FrameBundlePtr   FrameBundlePtr;

typedef mivins::ChannelImu       ImuHandler;
typedef mivins::ChannelOdom      OdomHandler;

typedef mivins::ImuMeasurement   ImuMeasurement;
typedef mivins::ImuMeasurements  ImuMeasurements;

typedef mivins::OdomMeasurement  OdomMeasurement;
typedef mivins::OdomMeasurements OdomMeasurements;


typedef mivins::Camera           Camera;
typedef mivins::CameraBundle     CameraBundle;
typedef mivins::CameraBundlePtr  CameraBundlePtr; 

typedef mivins::FeatureType      FeatureType;

/******************************Pose******************************/

void normalize(Transformation &T);

Transformation inverse(const Transformation T);

double getTimestamp(const FrameBundlePtr& frame_bundle);

Transformation getImu2WorldTrans(const FrameBundlePtr& frame_bundle);

Transformation getWorld2ImuTrans(const FrameBundlePtr& frame_bundle);

Eigen::Matrix3d getImu2WorldRot(const FrameBundlePtr& frame_bundle);

Eigen::Vector3d getImu2WorldPos(const FrameBundlePtr& frame_bundle);

Eigen::Matrix3d getRotationMatrix(const Transformation& T);

Quaternion getRotation(const Transformation& T);

Eigen::Vector3d getPosition(const Transformation& T);

Eigen::Matrix4d getTransformationMatrix(const Transformation& T);

void setImu2WorldTrans(const FrameBundlePtr& frame_bundle, Transformation &T);


/******************************Point******************************/
bool isPointVisible(const FramePtr &frame, const PointPtr& point);

/******************************camera******************************/
Camera::Type getCamType(const FramePtr &frame);

/******************************Frame******************************/
bool isKeyframe(const FramePtr &frame);

bool isKeyframe(const FrameBundlePtr &frame);

bool cornerFeature(const FeatureType &type);

bool edgeletFeature(const FeatureType &type);

int getFrameId(const FramePtr &frame);

int getNFrameIndex(const FramePtr &frame);

int getPointId(const PointPtr& point);

size_t numFeatures(const FramePtr &frame);

size_t numLandmarks(const FramePtr &frame);

void setKeyPoints(const FramePtr &frame);

void resetKeyPoints(const FramePtr &frame);

double getErrorMultiplier(const FramePtr &frame);

size_t numFeatures(const FrameBundlePtr &frame_bundle);

Eigen::Vector3d getLandmarkPos(const PointPtr &landmark);

void setLandmarkPos(PointPtr &landmark, Eigen::Vector3d &pos);

void removeLandmarkObs(const PointPtr &landmark, const FramePtr &frame);

void addLandmarkObs(const PointPtr &landmark, const FramePtr &frame, int feat_idx);

void pointOptimize(const PointPtr &point, int max_iter, bool optimize_on_sphere);

/******************************Imu******************************/
bool waitTill(const std::shared_ptr<ImuHandler> &imu_handler, double timestamp);

bool getMeasurementsContainingEdges(const std::shared_ptr<ImuHandler> &imu_handler, 
          double timestamp, ImuMeasurements& imu_measurements, bool erase);

/******************************Odom******************************/
bool waitTill(const std::shared_ptr<OdomHandler> &odom_handler, double timestamp);

bool getMeasurementsContainingEdges(const std::shared_ptr<OdomHandler> &odom_handler, 
          double timestamp, OdomMeasurements& odom_measurements, bool erase);

}
