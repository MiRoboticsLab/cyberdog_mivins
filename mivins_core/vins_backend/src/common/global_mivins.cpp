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

#include "common/global_mivins.h"

namespace backend
{

void normalize(Transformation &T)
{
  T.GetRotation().Normalize();
}

Transformation inverse(const Transformation T)
{
  return T.Inverse();
}

double getTimestamp(const FrameBundlePtr& frame_bundle)
{
  assert(frame_bundle != nullptr);
  return frame_bundle->GetMinTimestampSeconds();
}

Transformation getImu2WorldTrans(const FrameBundlePtr& frame_bundle)
{
  return frame_bundle->Get_T_W_B();
}

Transformation getWorld2ImuTrans(const FrameBundlePtr& frame_bundle)
{
  return frame_bundle->Get_T_W_B().Inverse();
}

Eigen::Matrix3d getImu2WorldRot(const FrameBundlePtr& frame_bundle)
{
  return frame_bundle->Get_T_W_B().GetRotationMatrix();
}

Eigen::Vector3d getImu2WorldPos(const FrameBundlePtr& frame_bundle)
{
  return frame_bundle->Get_T_W_B().GetPosition();
}

Eigen::Matrix3d getRotationMatrix(const Transformation& T)
{
  return T.GetRotationMatrix();
}

Quaternion getRotation(const Transformation& T)
{
  return T.GetRotation();
}

Eigen::Vector3d getPosition(const Transformation& T)
{
  return T.GetPosition();
}

Eigen::Matrix4d getTransformationMatrix(const Transformation& T)
{
  return T.GetTransformationMatrix();
}

void setImu2WorldTrans(const FrameBundlePtr& frame_bundle, Transformation &T)
{
  T.GetRotation().Normalize();
  frame_bundle->Set_T_W_B(T);
} 


/******************************Point******************************/
bool isPointVisible(const FramePtr &frame, const PointPtr& point)
{
  return frame->IsVisible(point->pos());
}

/******************************camera******************************/
Camera::Type getCamType(const FramePtr &frame)
{
  return frame->cam()->GetType();
}

/******************************Frame******************************/
bool isKeyframe(const FramePtr &frame)
{
  return frame->IsKeyframe();
}

bool isKeyframe(const FrameBundlePtr &frame_bundle)
{
  if(frame_bundle->IsKeyframe())
    return true;

  size_t cam_size = frame_bundle->size();
  for(size_t i = 0; i < cam_size; ++i)
  {
    const FramePtr frame = frame_bundle->at(i);
    if(isKeyframe(frame)) return true;
  }

  return false;
}

bool cornerFeature(const FeatureType &type)
{
  return mivins::isCorner(type);
}

bool edgeletFeature(const FeatureType &type)
{
  return mivins::isEdgelet(type);
}


int getFrameId(const FramePtr &frame)
{
  return frame->GetFrameId();
}

int getNFrameIndex(const FramePtr &frame)
{
  return frame->GetNFrameIndex();
}

int getPointId(const PointPtr& point)
{
  return point->Id();
}

size_t numFeatures(const FramePtr &frame)
{
  return frame->NumFeatures();
}

size_t numLandmarks(const FramePtr &frame)
{
  return frame->NumLandmarks();
}

void setKeyPoints(const FramePtr &frame)
{
  // frame->SetKeyPoints();
}

void resetKeyPoints(const FramePtr &frame)
{
  // frame->ResetKeyPoints();
}

double getErrorMultiplier(const FramePtr &frame)
{
  return frame->GetErrorMultiplier();
}

size_t numFeatures(const FrameBundlePtr &frame_bundle)
{
  return frame_bundle->NumFeatures();
}

Eigen::Vector3d getLandmarkPos(const PointPtr &landmark)
{
  return landmark->pos3d_in_w;
}

void setLandmarkPos(PointPtr &landmark, Eigen::Vector3d &pos)
{
  if(!landmark) return;
  
  landmark->pos3d_in_w = pos;
}

void removeLandmarkObs(const PointPtr &landmark, const FramePtr &frame)
{
  landmark->RemoveObservation(frame->GetFrameId());
}

void addLandmarkObs(const PointPtr &landmark, const FramePtr &frame, int feat_idx)
{
  landmark->AddObservation(frame, feat_idx);
}

void pointOptimize(const PointPtr &point, int max_iter, bool optimize_on_sphere)
{
  point->Optimize(max_iter, optimize_on_sphere);
}

/******************************Imu******************************/
bool waitTill(const std::shared_ptr<ImuHandler> &imu_handler, double timestamp)
{
  return imu_handler->WaitTill(timestamp);
}

bool getMeasurementsContainingEdges(const std::shared_ptr<ImuHandler> &imu_handler, 
  double timestamp, ImuMeasurements& imu_measurements, bool erase)
{
  return imu_handler->GetMeasurementsContainingEdges(timestamp, imu_measurements, erase);
}


/******************************Odom******************************/
bool waitTill(const std::shared_ptr<OdomHandler> &odom_handler, double timestamp)
{
  return odom_handler->WaitTill(timestamp);
}

bool getMeasurementsContainingEdges(const std::shared_ptr<OdomHandler> &odom_handler, 
  double timestamp, OdomMeasurements& odom_measurements, bool erase)
{
  return odom_handler->GetMeasurementsContainingEdges(timestamp, odom_measurements, erase);
}

}
