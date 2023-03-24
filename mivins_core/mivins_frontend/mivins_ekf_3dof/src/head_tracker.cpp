/*
 * Copyright 2019 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "mivins/ekf_3dof/head_tracker.h"

#include "mivins/ekf_3dof/neck_model.h"
#include "mivins/ekf_3dof/pose_prediction.h"
#include "mivins/ekf_3dof/logging.h"
#include "mivins/ekf_3dof/vector.h"
#include "mivins/ekf_3dof/vectorutils.h"

namespace ekf3dof {

HeadTracker::HeadTracker()
    : is_tracking_(false),
      sensor_fusion_(new SensorFusionEkf()),
      latest_gyroscope_data_({0, 0, Vector3::Zero()})
      //accel_sensor_(new SensorEventProducer<AccelerometerData>()),
      //gyro_sensor_(new SensorEventProducer<GyroscopeData>()) 
      {
  sensor_fusion_->SetBiasEstimationEnabled(/*kGyroBiasEstimationEnabled*/ true);
  // on_accel_callback_ = [&](const AccelerometerData& event) {
  //   OnAccelerometerData(event);
  // };
  // on_gyro_callback_ = [&](const GyroscopeData& event) {
  //   OnGyroscopeData(event);
  // };
}

HeadTracker::~HeadTracker() { }//UnregisterCallbacks(); }

void HeadTracker::Pause() {
  if (!is_tracking_) {
    return;
  }

  //UnregisterCallbacks();

  // Create a gyro event with zero velocity. This effectively stops the
  // prediction.
  GyroscopeData event = latest_gyroscope_data_;
  event.data = Vector3::Zero();

  OnGyroscopeData(event);

  is_tracking_ = false;
}

void HeadTracker::Resume() {
  //CARDBOARD_LOGI("WZJ HeadTracker Resume");
  is_tracking_ = true;
  //RegisterCallbacks();
}

bool HeadTracker::GetPose(int64_t timestamp_ns,
                          std::array<float, 3>& out_position,
                          std::array<float, 4>& out_orientation) const {
  Rotation predicted_rotation;
  bool filter_nice = false;
  const PoseState pose_state = sensor_fusion_->GetLatestPoseState();
  if (sensor_fusion_->IsFullyInitialized()) {
    predicted_rotation = pose_prediction::PredictPose(timestamp_ns, pose_state);
    filter_nice = true;
  } else {
    predicted_rotation = pose_state.sensor_from_start_rotation;
    filter_nice = false;
  }
    //   #### BOTH OF IMU AND CAMERA ARE RIGHT HAND SYSTEM ####
    //  IMU coordinate system           Camera coordinate system
    //     _________________              _________________
    //    !                !             !                !
    //    !    Y           !             !                !
    //    !    !           !             !                !
    //    !    !           !             !  Y  ------* Z  !
    //    !    !           !             !           !    !
    //    !    . ______ X  !             !           !    !
    //    !   Z            !             !           !    !
    //    !                !             !           X    !
    //    !________________!             !________________!
    //    !   <-  O   ->   !             !   <-  O   ->   !
    //    !________________!             !________________!
  // In order to update our pose as the sensor changes, we begin with the
  // inverse default orientation (the orientation returned by a reset sensor),
  // apply the current sensor transformation, and then transform into display
  // space.
  // TODO(b/135488467): Support different screen orientations.
  const Rotation ekf_to_head_tracker =
      Rotation::FromYawPitchRoll(-M_PI / 2.0, 0, -M_PI / 2.0);
  const Rotation sensor_to_display =
      Rotation::FromAxisAndAngle(Vector3(0, 0, 1), M_PI / 2.0);

  const Vector4 q = predicted_rotation.GetQuaternion();
     // (sensor_to_display * predicted_rotation * ekf_to_head_tracker)
     //     .GetQuaternion();
  Rotation rotation;
  rotation.SetQuaternion(q);

  out_orientation[0] = static_cast<float>(rotation.GetQuaternion()[0]);
  out_orientation[1] = static_cast<float>(rotation.GetQuaternion()[1]);
  out_orientation[2] = static_cast<float>(rotation.GetQuaternion()[2]);
  out_orientation[3] = static_cast<float>(rotation.GetQuaternion()[3]);

  out_position[0] = pose_state.position[0];
  out_position[1] = pose_state.position[1];
  out_position[2] = pose_state.position[2];
  return filter_nice;
  //out_position = ApplyNeckModel(out_orientation, 1.0);
}

Rotation HeadTracker::GetDefaultOrientation() const {
  return Rotation::FromRotationMatrix(
      Matrix3x3(0.0, -1.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0));
}

// void HeadTracker::RegisterCallbacks() {
//   accel_sensor_->StartSensorPolling(&on_accel_callback_);
//   gyro_sensor_->StartSensorPolling(&on_gyro_callback_);
// }

// void HeadTracker::UnregisterCallbacks() {
//   accel_sensor_->StopSensorPolling();
//   gyro_sensor_->StopSensorPolling();
// }

void HeadTracker::OnAccelerometerData(const AccelerometerData& event) {
    //   #### BOTH OF IMU AND CAMERA ARE RIGHT HAND SYSTEM ####
    //  IMU coordinate system           Camera coordinate system
    //     _________________              _________________
    //    !                !             !                !
    //    !    Y           !             !                !
    //    !    !           !             !                !
    //    !    !           !             !  Y  ------* Z  !
    //    !    !           !             !           !    !
    //    !    . ______ X  !             !           !    !
    //    !   Z            !             !           !    !
    //    !                !             !           X    !
    //    !________________!             !________________!
    //    !   <-  O   ->   !             !   <-  O   ->   !
    //    !________________!             !________________!
  if (!is_tracking_) {
    return;
  }
  sensor_fusion_->ProcessAccelerometerSample(event);
}

void HeadTracker::OnGyroscopeData(const GyroscopeData& event) {
    //   #### BOTH OF IMU AND CAMERA ARE RIGHT HAND SYSTEM ####
    //  IMU coordinate system           Camera coordinate system
    //     _________________              _________________
    //    !                !             !                !
    //    !    Y           !             !                !
    //    !    !           !             !                !
    //    !    !           !             !  Y  ------* Z  !
    //    !    !           !             !           !    !
    //    !    . ______ X  !             !           !    !
    //    !   Z            !             !           !    !
    //    !                !             !           X    !
    //    !________________!             !________________!
    //    !   <-  O   ->   !             !   <-  O   ->   !
    //    !________________!             !________________!
  if (!is_tracking_) {
    return;
  }
  latest_gyroscope_data_ = event;
  sensor_fusion_->ProcessGyroscopeSample(event);
}

}  // namespace ekf3dof
