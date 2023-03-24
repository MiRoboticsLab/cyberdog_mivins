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

#include "vins_backend_interface.h"

namespace mivins
{

VinsBackendInterface::VinsBackendInterface(    
  const VinsBackendOptions& backend_options,
  const VinsMotionDetectorOptions& motion_detector_options,
  const CameraBundlePtr& camera_bundle, const int pipeline_type)
{
  get_motion_prior_with_imu_ = backend_options.get_motion_prior_with_imu;
  get_motion_prior_with_odom_ = backend_options.get_motion_prior_with_odom;
  get_motion_prior_with_odompose_ = backend_options.get_motion_prior_with_odompose;

  setParameters(backend_options, pipeline_type);

  int cam_size = camera_bundle->getNumCameras();
  vins_estimator_ = std::make_shared<Estimator>(camera_bundle, backend_options);
  vins_estimator_->createMotionDetector(motion_detector_options);

  outlier_rejection_ = std::make_shared<VinsOutlierRejection>();

  g_ = Eigen::Vector3d(0.0, 0.0, 9.81);
  latest_state_.P = Eigen::Vector3d::Zero();
  latest_state_.V = Eigen::Vector3d::Zero();
  latest_state_.R = Eigen::Matrix3d::Identity();

  latest_state_.Ba = Eigen::Vector3d::Zero();
  latest_state_.Bg = Eigen::Vector3d::Zero();

  input_imu_ts_prv_ = 0.0f;
  input_odom_ts_prv_ = 0.0f;
}

VinsBackendInterface::~VinsBackendInterface()
{
  reset();
}

void VinsBackendInterface::reset()
{
  vins_estimator_->reset();

  // vins_estimator_->createMotionDetector(motion_detector_options);

  outlier_rejection_ = std::make_shared<VinsOutlierRejection>();
  g_ = Eigen::Vector3d(0.0, 0.0, 9.81);

  latest_state_.P.setZero();
  latest_state_.V.setZero();
  latest_state_.R.setIdentity();
  latest_state_.Ba.setZero();
  latest_state_.Bg.setZero();

  input_imu_ts_prv_ = 0.0f;
  input_odom_ts_prv_ = 0.0f;
}

int VinsBackendInterface::getWindowsSize()
{
  return WINDOW_SIZE;
}

bool VinsBackendInterface::isEstimatorValid()
{
  return vins_estimator_ != nullptr;
}

FrameBundlePtr VinsBackendInterface::getFrameBundle(int i)
{
  if(i > WINDOW_SIZE) return nullptr;
  return vins_estimator_->states_[i].frame_bundle;
}

void VinsBackendInterface::setImuHandler(
    const std::shared_ptr<ImuHandler> imu_handler)
{
  imu_handler_ = imu_handler;
  vins_estimator_->setImuHandler(imu_handler);
}

void VinsBackendInterface::setOdomHandler(
    const std::shared_ptr<OdomHandler> odom_handler)
{
  odom_handler_ = odom_handler;
  vins_estimator_->setOdomHandler(odom_handler);
}

FrameBundlePtr VinsBackendInterface::getRefFrames()
{
  return vins_estimator_->init_ref_frames_;
  // return ref_frames_;
}

void VinsBackendInterface::getMotionPrior(const FrameBundlePtr& new_frames, 
  const FrameBundlePtr& last_frames, bool &have_motion_prior)
{
  bool res = false;
  have_motion_prior = false;

  if(get_motion_prior_with_odom_ && odom_handler_)
  {
    getMotionPriorWithOdom(new_frames, last_frames, have_motion_prior);
    if(have_motion_prior)
      return;
  }
  
  if(get_motion_prior_with_imu_ && imu_handler_)
  {
    getMotionPriorWithImu(new_frames, last_frames, have_motion_prior);
    if(have_motion_prior)
      return;
  }
}

void VinsBackendInterface::getMotionPriorWithImu(const FrameBundlePtr& new_frames, 
  const FrameBundlePtr& last_frames, bool &have_motion_prior)
{
  if(!imu_handler_ || !last_frames)
    return;

  if(vins_estimator_->solver_flag_ == SolverFlag::INITIAL)
  {
    have_motion_prior = false;
    return;
  }

  bool res = false;
  ImuMeasurements imu_measurements;
  const double cur_ts = getTimestamp(new_frames);
  const double prv_ts = getTimestamp(last_frames);
  res = getImuMeasurements(cur_ts, imu_measurements, false);
  if(!res)
  {
    printf("VinsBackendInterface::getMotionPriorWithImu: getImuMeasurements failed\n");
    have_motion_prior = false;
    return;
  }

  double td_imu = vins_estimator_->getTdImu();
  res = predictWithImu(imu_measurements, prv_ts + td_imu, cur_ts + td_imu);
  if(!res)
  {
    printf("VinsBackendInterface::getMotionPriorWithImu: predictWithImu failed\n");
    have_motion_prior = false;
    return;
  }

  const Eigen::Matrix3d &latest_R = latest_state_.R;
  const Eigen::Vector3d &latest_P = latest_state_.P;
  Eigen::Quaterniond latest_Q(latest_R);
  latest_Q.normalize();

  Transformation T_WS(latest_Q, latest_P);
  setImu2WorldTrans(new_frames, T_WS);
  have_motion_prior = true;
}

void VinsBackendInterface::getMotionPriorWithOdom(const FrameBundlePtr& new_frames, 
  const FrameBundlePtr& last_frames, bool& have_motion_prior)
{
  if(!odom_handler_ || !last_frames)
    return;

  if(vins_estimator_->solver_flag_ == SolverFlag::INITIAL)
  {
    have_motion_prior = false;
    return;
  }

  bool res = false;
  OdomMeasurements odom_measurements;
  double td_odom = vins_estimator_->getTdOdom();
  const double cur_ts = getTimestamp(new_frames);
  const double prv_ts = getTimestamp(last_frames);
  res = getOdomMeasurements(cur_ts, odom_measurements, false);
  if(!res)
  {
    printf("VinsBackendInterface::getMotionPriorWithOdom: getImuMeasurements failed\n");
    have_motion_prior = false;
    return;
  }

  res = predictWithOdom(odom_measurements, prv_ts + td_odom, cur_ts + td_odom);
  if(!res)
  {
    printf("VinsBackendInterface::getMotionPriorWithOdom: predictWithOdom failed\n");
    have_motion_prior = false;
    return;
  }
  
  const Eigen::Matrix3d &latest_R = latest_state_.R;
  const Eigen::Vector3d &latest_P = latest_state_.P;
  Eigen::Quaterniond latest_Q(latest_R);
  latest_Q.normalize();

  Transformation T_WS(latest_Q, latest_P);
  setImu2WorldTrans(new_frames, T_WS);
  have_motion_prior = true;
}

void VinsBackendInterface::alignImuAndOdom(
  ImuMeasurements &imu_measurements, 
  OdomMeasurements &odom_measurements)
{
  int imu_idx = (int)imu_measurements.size() - 1;
  int odom_idx = (int)odom_measurements.size() - 1;

  const Eigen::Matrix3d &roi = vins_estimator_->roi_;

  for(; odom_idx >= 0; --odom_idx)
  {
    double imu_ts_cur;
    double imu_ts_prv;
    Eigen::Vector3d imu_gyr_cur = Eigen::Vector3d::Zero();
    Eigen::Vector3d imu_gyr_prv = Eigen::Vector3d::Zero();
    double odom_ts_cur = odom_measurements[odom_idx].timestamp_;
    
    for(; imu_idx >= 0; --imu_idx)
    {
      if(imu_measurements[imu_idx].timestamp_ >= odom_ts_cur)
        break;
    }

    imu_ts_cur = imu_measurements[imu_idx < 0 ? 0 : imu_idx].timestamp_;
    imu_gyr_cur = imu_measurements[imu_idx < 0 ? 0 : imu_idx].angular_velocity_;

    Eigen::Vector3d imu_gyr = Eigen::Vector3d::Zero();
    if(imu_idx == (int)imu_measurements.size() - 1 || imu_idx < 0)
      imu_gyr = imu_gyr_cur; 
    else if(imu_idx != (int)imu_measurements.size() - 1)
    {
      imu_ts_prv = imu_measurements[imu_idx + 1].timestamp_;
      imu_gyr_prv = imu_measurements[imu_idx + 1].angular_velocity_;

      vins_estimator_->valueInterpolation(
          imu_ts_prv, imu_gyr_prv, 
          imu_ts_cur, imu_gyr_cur, 
          odom_ts_cur, imu_gyr);
    }
    
    // std::cout << "odom idx: " << odom_idx << ": ";
    // std::cout << "GYR: " << (R_O_B * imu_gyr).transpose() << "; "
    //           << odom_measurements[odom_idx].angular_velocity_.transpose() << "\n";

    odom_measurements[odom_idx].angular_velocity_ = roi * imu_gyr;
  }

}

bool VinsBackendInterface::predictWithImu(
  const ImuMeasurements& imu_measurements, 
  const double ts_start, const double ts_end)
{
  bool res = false;
  res = predictWithImuMedian(imu_measurements, ts_start, ts_end);
  // res = predictWithImuRK4(imu_measurements, ts_start, ts_end);
  return res;
}

bool VinsBackendInterface::predictWithImuMedian(
  const ImuMeasurements& imu_measurements, 
  const double ts_start, const double ts_end)
{
  if(imu_measurements.back().timestamp_ > ts_start)
  {
    printf("The timestamp of imu should less than or equal to start time\n");
    printf("new imu ts: %.9f, image end ts: %.9f\n", imu_measurements.back().timestamp_, ts_start);
    return false;
  }
  if(imu_measurements.front().timestamp_ < ts_end)
  {
    printf("The timestamp of imu should big than or equal to start time\n");
    printf("new imu ts: %.9f, image end ts: %.9f\n", imu_measurements.front().timestamp_, ts_end);
    return false;
  }

  double ts_cur, ts_prv;
  Eigen::Vector3d imu_acc_cur, imu_acc_prv;
  Eigen::Vector3d imu_gyr_cur, imu_gyr_prv;

  for(int i = (int)imu_measurements.size() - 1; i >= 0; --i)
  {
    ts_cur = imu_measurements[i].timestamp_;
    imu_gyr_cur = imu_measurements[i].angular_velocity_;
    imu_acc_cur = imu_measurements[i].linear_acceleration_;

    double dt = 0.0f;
    if(ts_cur <= ts_start)
    {
      imu_acc_prv = imu_acc_cur;
      imu_gyr_prv = imu_gyr_cur;
      ts_prv = ts_cur;
      continue;
    }
    else if(ts_cur >= ts_end)
      dt = ts_end - ts_prv;
    else 
      dt = ts_cur - ts_prv;
    
    Eigen::Vector3d un_acc_0 = latest_state_.R * (imu_acc_prv - latest_state_.Ba) - g_;
    Eigen::Vector3d un_gyr = 0.5 * (imu_gyr_prv + imu_gyr_cur) - latest_state_.Bg;
    latest_state_.R = latest_state_.R * Utility::deltaQ(un_gyr * dt).toRotationMatrix();
    Eigen::Vector3d un_acc_1 = latest_state_.R * (imu_acc_cur - latest_state_.Ba) - g_;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    latest_state_.P = latest_state_.P + dt * latest_state_.V + 0.5 * dt * dt * un_acc;
    latest_state_.V = latest_state_.V + dt * un_acc;

    imu_acc_prv = imu_acc_cur;
    imu_gyr_prv = imu_gyr_cur;
    ts_prv = ts_cur;
  }

  // std::cout << "---: " << latest_state_.P.transpose() << std::endl;

  return true;
}

bool VinsBackendInterface::predictWithImuRK4(
  const ImuMeasurements& imu_measurements, 
  const double ts_start, const double ts_end)
{
  if(imu_measurements.back().timestamp_ > ts_start)
  {
    printf("The timestamp of imu should less than or equal to start time\n");
    printf("new imu ts: %.9f, image end ts: %.9f\n", imu_measurements.back().timestamp_, ts_start);
    return false;
  }
  if(imu_measurements.front().timestamp_ < ts_end)
  {
    printf("The timestamp of imu should big than or equal to start time\n");
    printf("new imu ts: %.9f, image end ts: %.9f\n", imu_measurements.front().timestamp_, ts_end);
    return false;
  }

  double dt_prv;
  double ts_cur, ts_prv;
  Eigen::Vector3d imu_acc_cur, imu_acc_prv;
  Eigen::Vector3d imu_gyr_cur, imu_gyr_prv;

  for(int i = (int)imu_measurements.size() - 1; i >= 0; --i)
  {
    ts_cur = imu_measurements[i].timestamp_;
    imu_gyr_cur = imu_measurements[i].angular_velocity_;
    imu_acc_cur = imu_measurements[i].linear_acceleration_;

    double dt = 0.0f;
    if(ts_cur <= ts_start)
    {
      imu_acc_prv = imu_acc_cur;
      imu_gyr_prv = imu_gyr_cur;
      ts_prv = ts_cur;
      dt_prv = dt;
      continue;
    }
    else if(ts_cur >= ts_end)
      dt = ts_end - ts_prv;
    else 
      dt = ts_cur - ts_prv;

    // 4 Runge Kutta Process
    Eigen::Vector3d acc_predicted1, acc_predicted2;
    Eigen::Vector3d	gyr_predicted1, gyr_predicted2;
    if(dt_prv < 0.000001)
    {
      acc_predicted1 = imu_acc_cur;
      acc_predicted2 = imu_acc_cur;
      gyr_predicted1 = imu_gyr_cur;
      gyr_predicted2 = imu_gyr_cur;
    }
    else
    {
      acc_predicted1 = (imu_acc_cur - imu_acc_prv) * dt / dt_prv * 0.5 + imu_acc_cur;
      acc_predicted2 = (imu_acc_cur - imu_acc_prv) * dt / dt_prv * 1.0 + imu_acc_cur;
      gyr_predicted1 = (imu_gyr_cur - imu_gyr_prv) * dt / dt_prv * 0.5 + imu_gyr_cur;
      gyr_predicted2 = (imu_gyr_cur - imu_gyr_prv) * dt / dt_prv * 1.0 + imu_gyr_cur;
    }

    // k1
    Eigen::Vector3d k1_r = imu_gyr_cur * dt;
    Eigen::Vector3d k1_v = (latest_state_.R * imu_acc_cur - g_) * dt;
    Eigen::Vector3d k1_p = k1_v * dt;

    latest_state_.R *= expMatrix(k1_r * 0.5);
    latest_state_.V += latest_state_.R * JacobianL(k1_r * 0.5) * latest_state_.R.transpose() * (k1_v * 0.5);
    latest_state_.P += (k1_p * 0.5) + 0.5 * dt * (k1_v * 0.5);
    
    // k2
    Eigen::Vector3d k2_r = gyr_predicted1 * dt;
    Eigen::Vector3d k2_v = (latest_state_.R * acc_predicted1 - g_) * dt;
    Eigen::Vector3d k2_p = k2_v * dt;

    latest_state_.R *= expMatrix(k2_r * 0.5);
    latest_state_.V += latest_state_.R * JacobianL(k2_r * 0.5) * latest_state_.R.transpose() * (k2_v * 0.5);
    latest_state_.P += (k2_p * 0.5) + 0.5 * dt * (k2_v * 0.5);

    // k3
    Eigen::Vector3d k3_r = gyr_predicted1 * dt;
    Eigen::Vector3d k3_v = (latest_state_.R * acc_predicted1 - g_) * dt;
    Eigen::Vector3d k3_p = k3_v * dt;

    latest_state_.R *= expMatrix(k3_r * 1.0);
    latest_state_.V += latest_state_.R * JacobianL(k3_r * 1.0) * latest_state_.R.transpose() * (k3_v * 1.0);
    latest_state_.P += (k3_p * 1.0) + 0.5 * dt * (k3_v * 1.0);

    // k4
    Eigen::Vector3d k4_r = gyr_predicted2 * dt;
    Eigen::Vector3d k4_v = (latest_state_.R * acc_predicted2 - g_) * dt;
    Eigen::Vector3d k4_p = k4_v * dt;

    Eigen::Vector3d tmp_r = 1.0/6 * k1_r + 2.0/6 * k2_r + 2.0/6 * k3_r + 1.0/6 * k4_r;
    Eigen::Vector3d tmp_v = 1.0/6 * k1_v + 2.0/6 * k2_v + 2.0/6 * k3_v + 1.0/6 * k4_v;
    Eigen::Vector3d tmp_p = 1.0/6 * k1_p + 2.0/6 * k2_p + 2.0/6 * k3_p + 1.0/6 * k4_p;

    latest_state_.R *= expMatrix(tmp_r * 1.0);
    latest_state_.V += latest_state_.R * JacobianL(tmp_r * 1.0) * latest_state_.R.transpose() * (tmp_v * 1.0);
    latest_state_.P += (tmp_p * 1.0) + 0.5 * dt * (tmp_v * 1.0);
  
    imu_acc_prv = imu_acc_cur;
    imu_gyr_prv = imu_gyr_cur;
    ts_prv = ts_cur;
    dt_prv = dt;
  }

  return true;
}

bool VinsBackendInterface::predictWithOdom(
  const OdomMeasurements& odom_measurements, 
  const double ts_start, const double ts_end)
{
  if(odom_measurements.back().timestamp_ > ts_start)
  {
    printf("The timestamp of odom should less than or equal to start time\n");
    printf("new odom start ts: %.9f, image start ts: %.9f\n", odom_measurements.back().timestamp_, ts_start);
    return false;
  }
  if(odom_measurements.front().timestamp_ < ts_end)
  {
    printf("The timestamp of odom should big than or equal to start time\n");
    printf("new odom end ts: %.9f, image end ts: %.9f\n", odom_measurements.front().timestamp_, ts_end);
    return false;
  }

  const Eigen::Matrix3d &rio = vins_estimator_->rio_;
  const Eigen::Vector3d &tio = vins_estimator_->tio_;

  const Eigen::Matrix3d &roi = vins_estimator_->roi_;
  const Eigen::Vector3d &toi = vins_estimator_->toi_;

  Eigen::Matrix3d R_w_o = latest_state_.R * rio;
  Eigen::Vector3d t_w_o = latest_state_.R * tio + latest_state_.P;

  double ts_prv, ts_cur;
  Eigen::Vector3d odom_vel_prv, odom_vel_cur;
  Eigen::Vector3d odom_gyr_prv, odom_gyr_cur;

  if(get_motion_prior_with_odompose_)
  {
    std::cout<<"get_motion_prior_with_odompose_"<<std::endl;
    int n = odom_measurements.size() - 1;
    Eigen::Vector3d odom_pos0 = odom_measurements[n].position_;
    Eigen::Quaterniond odom_rot0 = odom_measurements[n].orientation_;
    const float pi = 3.1415926535;

    // std::cout<<"yaw pitch roll(zyx 0) = "<<odom_rot0.matrix().eulerAngles(2,1,0).transpose()*180/pi<<std::endl;

    // std::cout<< "odom timen:" << std::fixed << std::setprecision(5) << odom_measurements[n].timestamp_<<std::endl;
    // std::cout.unsetf( std::ios::fixed ); 
    // std::cout<< odom_pos0[0] << " " << odom_pos0[1] << " " << odom_pos0[2] << " ; "
    //       << odom_rot0.x() << " "<< odom_rot0.y() << " " << odom_rot0.z() << " " << odom_rot0.w() << std::endl; 
    Eigen::Vector3d odom_posn = odom_measurements[0].position_;
    Eigen::Quaterniond odom_rotn = odom_measurements[0].orientation_;
    // std::cout<<"yaw pitch roll(zyx n) = "<<odom_rotn.matrix().eulerAngles(2,1,0).transpose()*180/pi<<std::endl;
    // std::cout<< "odom time0:" << std::fixed << std::setprecision(5) <<odom_measurements[0].timestamp_<<std::endl;
    // std::cout.unsetf( std::ios::fixed ); 
    // std::cout<< odom_posn[0] << " " << odom_posn[1] << " " << odom_posn[2] << " ; "
    //       << odom_rotn.x() << " "<< odom_rotn.y() << " " << odom_rotn.z() << " " << odom_rotn.w() << std::endl; 
    
    odom_pos0[2] = 0.0;
    odom_posn[2] = 0.0;
    Transformation T_o1, T_on;
    T_o1 = Transformation(odom_pos0, odom_rot0);
    T_on = Transformation(odom_posn, odom_rotn);
    Transformation T_o1n = T_o1.Inverse() * T_on;
    // Eigen::Quaterniond q_o1n = T_o1n.GetRotation().ToImplementation();
    // std::cout<<"yaw pitch roll(zyx d) = "<<q_o1n.matrix().eulerAngles(2,1,0).transpose()*180/pi<<std::endl;
    // Eigen::Vector3d eulerAngle=q_o1n.matrix().eulerAngles(2,1,0);
    // Eigen::AngleAxisd rotation_vector(eulerAngle[0], Eigen::Vector3d(0, 0, 1));
    // Eigen::Matrix3d rotation_matrix = rotation_vector.matrix();
    
    // std::cout<<"d(zyx d) = "<< T_o1n.GetPosition().transpose()<<std::endl;
    // Transformation T_o1n_new(Quaternion(rotation_matrix),T_o1n.GetPosition());
    Transformation T_wo1(Quaternion(R_w_o),t_w_o);
    Transformation T_oi(Quaternion(roi),toi);

    Transformation T_wb = T_wo1*T_o1n*T_oi;
    Eigen::Matrix3d r = T_wb.GetRotation().GetRotationMatrix();
    Eigen::Vector3d p = T_wb.GetPosition();
    
    latest_state_.R = r;
    latest_state_.P = p;
    
    return true;
  }
  else
  {
    std::cout<<"get_motion_prior_with_odom_integration"<<std::endl;
    for(int i = (int)odom_measurements.size() - 1; i >= 0; --i)
    {
      ts_cur = odom_measurements[i].timestamp_;
      odom_vel_cur = odom_measurements[i].linear_velocity_;
      odom_gyr_cur = odom_measurements[i].angular_velocity_;

      double dt = 0.0f;
      if(ts_cur <= ts_start)
      {
        odom_vel_prv = odom_vel_cur;
        odom_gyr_prv = odom_gyr_cur;
        ts_prv = ts_cur;
        continue;
      }
      else if(ts_cur >= ts_end)
        dt = ts_end - ts_prv;
      else 
        dt = ts_cur - ts_prv;

      assert(dt >= 0.0f);

      Eigen::Vector3d odom_vel_0 = R_w_o * odom_vel_prv;
      Eigen::Vector3d un_odom_gyr = 0.5 * (odom_gyr_prv + odom_gyr_cur);
      R_w_o *= Utility::deltaQ(un_odom_gyr * dt).normalized().toRotationMatrix();

      Eigen::Vector3d odom_vel_1 = R_w_o * odom_vel_cur;
      t_w_o += 0.5 * (odom_vel_0 + odom_vel_1) * dt;

      odom_vel_prv = odom_vel_cur;
      odom_gyr_prv = odom_gyr_cur;
      ts_prv = ts_cur;
    }

    latest_state_.R = R_w_o * roi;
    latest_state_.P = R_w_o * toi + t_w_o;

    return true;
  }
  
}

bool VinsBackendInterface::getImuMeasurements(const double timestamp, 
                  ImuMeasurements& imu_measurements, bool erase)
{
  if(!imu_handler_)
    return false;

  const double td_imu = vins_estimator_->getTdImu();
  double timestamp_td = timestamp + td_imu;

  if (!waitTill(imu_handler_, timestamp_td))
  {
    printf("Couldn't get imu measurements, image ts: %.9f\n", timestamp_td);
    return false;
  }

  if (!getMeasurementsContainingEdges(imu_handler_, timestamp_td, imu_measurements, erase))
  {
    printf("Couldn't get imu measurements, image ts: %.9f\n", timestamp_td);
    return false;
  }

  return true;
}

bool VinsBackendInterface::getOdomMeasurements(const double timestamp, 
                  OdomMeasurements& odom_measurements, bool erase)
{
  if(!odom_handler_)
    return false;

  if (!waitTill(odom_handler_, timestamp))
  {
    printf("Couldn't get odom measurements, image ts: %.9f\n", timestamp);
    return false;
  }

  if (!getMeasurementsContainingEdges(odom_handler_, timestamp, odom_measurements, erase))
  {
    printf("Couldn't get odom measurements, image ts: %.9f\n", timestamp);
    return false;
  }

  return true;
}

bool VinsBackendInterface::tryToInitialize(const FrameBundlePtr& new_frames)
{
  vins_estimator_->addInitFrameBundle(new_frames);

  if(vins_estimator_->solver_flag_ == SolverFlag::NON_LINEAR)
  {
    for(int i = 0; i < 10; i++)
      std::cout << "Init done: NON_LINEAR\n";
   
    if(PIPLINE_TYPE == PipType::kMono || PIPLINE_TYPE == PipType::kRgbd)
      drawInitMonoMatches();
    else if(PIPLINE_TYPE == PipType::kStereo)
      drawInitStereoMatches();
    else if(PIPLINE_TYPE == PipType::kTripleWithDepth)
      drawInitMonoMatches();

    printInitResult();
    
    return true;
  }

  return false;
}

bool VinsBackendInterface::inputImu(const FrameBundlePtr& new_frames)
{
  if(!imu_handler_  || !vins_estimator_)
    return false;

  ImuMeasurements imu_measurements;
  double img_ts = getTimestamp(new_frames);
  bool res = getImuMeasurements(img_ts, imu_measurements, true);
  if(!res)
  {
    printf("VinsBackendInterface::inputImu: getImuMeasurements failed\n");
    return false;
  }

  for (int i = imu_measurements.size() - 1; i >= 0; --i)
  {
    double ts = imu_measurements[i].timestamp_;
    if(ts <= input_imu_ts_prv_)
      continue;
      
    // printf("imu: %d: %.9f %.9f\n", i, ts, input_imu_ts_prv_);
    const Eigen::Vector3d &angular_velocity = imu_measurements[i].angular_velocity_;
    const Eigen::Vector3d &linear_acceleration = imu_measurements[i].linear_acceleration_;
    ImuData imu_data(ts, linear_acceleration, angular_velocity);
    vins_estimator_->inputImu(imu_data);
    
    input_imu_ts_prv_ = ts;
  }

  return true;
}

bool VinsBackendInterface::inputOdom(const FrameBundlePtr& new_frames)
{
  // return inputImuAndOdom(new_frames);

  if(!odom_handler_ || !vins_estimator_)
    return false;

  OdomMeasurements odom_measurements;
  double img_ts = getTimestamp(new_frames);
  bool res = getOdomMeasurements(img_ts, odom_measurements, true);
  if(!res)
  {
    printf("VinsBackendInterface::inputOdom: getOdomMeasurements failed\n");
    return false;
  }

  for (int i = odom_measurements.size() - 1; i >= 0; --i)
  {
    const double &ts = odom_measurements[i].timestamp_;
    if(ts <= input_odom_ts_prv_)
      continue;

    const Eigen::Vector3d &odom_vel = odom_measurements[i].linear_velocity_;
    const Eigen::Vector3d &odom_gyr = odom_measurements[i].angular_velocity_;
    const Eigen::Vector3d &odom_pos = odom_measurements[i].position_;
    const Eigen::Quaterniond &odom_rot = odom_measurements[i].orientation_;

    OdomData odom_data(ts, odom_rot, odom_pos, odom_vel, odom_gyr);
    vins_estimator_->inputOdom(odom_data);
    input_odom_ts_prv_ = ts;
  }

  return true;
}

bool VinsBackendInterface::inputImuAndOdom(const FrameBundlePtr& new_frames)
{
  if(!odom_handler_ || !imu_handler_ || !vins_estimator_)
    return false;

  ImuMeasurements imu_measurements;
  OdomMeasurements odom_measurements;
  double img_ts = getTimestamp(new_frames);

  bool res_imu = getImuMeasurements(img_ts, imu_measurements, true);
  if(!res_imu)
  {
    printf("VinsBackendInterface::inputOdom: getImuMeasurements failed\n");
    return false;
  }

  bool res_odom = getOdomMeasurements(img_ts, odom_measurements, true);
  if(!res_odom)
  {
    printf("VinsBackendInterface::inputOdom: getOdomMeasurements failed\n");
    return false;
  }

  alignImuAndOdom(imu_measurements, odom_measurements);

  for (int i = odom_measurements.size() - 1; i >= 0; --i)
  {
    double ts = odom_measurements[i].timestamp_;
    if(ts <= input_odom_ts_prv_)
      continue;

    const Eigen::Vector3d &odom_vel = odom_measurements[i].linear_velocity_;
    const Eigen::Vector3d &odom_gyr = odom_measurements[i].angular_velocity_;
    const Eigen::Vector3d &odom_pos = odom_measurements[i].position_;
    const Eigen::Quaterniond &odom_rot = odom_measurements[i].orientation_;
    OdomData odom_data(ts, odom_rot, odom_pos, odom_vel, odom_gyr);
    vins_estimator_->inputOdom(odom_data);
    input_odom_ts_prv_ = ts;
  }

  return true;
}

bool VinsBackendInterface::addFrameBundle(const FrameBundlePtr& new_frames)
{
  if(vins_estimator_->solver_flag_ == SolverFlag::INITIAL)
  {
    printf("Wait for init done\n");
    return false;
  }
  
  vins_estimator_->addFrameBundle(new_frames);

  // std::string save_path = "/home/wangcp/debug/offline_data";
  // io_ptr_->addFrameData(save_path, new_frames);
 
  return true;
}

void VinsBackendInterface::updateLatestStates(
        const FrameBundlePtr& new_frames, 
        const FrameBundlePtr& last_frames)
{
  if(vins_estimator_->solver_flag_ == SolverFlag::INITIAL)
    return;

  if(!new_frames)
  {
    printf("updateLatestStates: new_frames is nullptr\n");
    return;
  }

  if(!last_frames)
  {
    printf("updateLatestStates: last_frames is nullptr\n");
    return;
  }

  bool update_with_be = false;
  double new_ts = getTimestamp(new_frames);
  double last_ts = getTimestamp(last_frames);

  {
    std::lock_guard<std::mutex> lock(vins_estimator_->state_mutex_);

    const auto &states = vins_estimator_->states_;
    for(int i = WINDOW_SIZE; i >= 0; --i)
    {
      const StateGroup &be_state = states[WINDOW_SIZE];
      if(!be_state.frame_bundle)
        continue;
        
      double be_ts = getTimestamp(be_state.frame_bundle);
      if(be_ts < last_ts)
        break;

      if(last_ts == be_ts)
      {
        update_with_be = true;

        latest_state_.P = be_state.P;
        latest_state_.R = be_state.R;
        if(vins_estimator_->use_imu_)
        {
          latest_state_.V = be_state.V;
          latest_state_.Ba = be_state.Ba;
          latest_state_.Bg = be_state.Bg;

          // imu_handler_->setGyroscopeBias(be_state.Bg);
          // imu_handler_->setAccelerometerBias(be_state.Ba);
        }
      }
    }
  }

  if(!update_with_be)
  {
    Transformation T = getImu2WorldTrans(last_frames);
    latest_state_.R = getRotationMatrix(T);
    latest_state_.P = getPosition(T);
  }
}

void VinsBackendInterface::updateLandmarks(const FrameBundlePtr& frame_bundle)
{
  if(!frame_bundle) return;

  auto &states = vins_estimator_->states_;
  auto &frame_manager = vins_estimator_->frame_manager_;

  size_t cam_size = frame_bundle->size();
  for(size_t cam_idx = 0; cam_idx < cam_size; ++cam_idx)
  {
    const FramePtr &frame = frame_bundle->at(cam_idx);
    size_t feature_size = numFeatures(frame);

    for(size_t ftr_idx = 0; ftr_idx < feature_size; ++ftr_idx)
    {
      if(ftr_idx >= (size_t)frame->track_id_vec_.size())
        continue;

      if(!frame->landmark_vec_[ftr_idx])
        continue;

      const int feature_id = frame->track_id_vec_(ftr_idx);
      if(frame_manager->feature_per_ids_.count(feature_id) == 0)
        continue;

      auto &feature_per_id = frame_manager->feature_per_ids_[feature_id];
      const int &solve_flag = feature_per_id.solve_flag;
      const double &estimated_depth = feature_per_id.estimated_depth;

      if(std::abs(solve_flag) != 1 || estimated_depth < 0.0f || estimated_depth == INIT_DEPTH)
        continue;

      auto &obs_frames = feature_per_id.obs_frames;
      auto &feature_per_frame = feature_per_id.feature_per_frame;

      int frame_id = obs_frames.front();
      auto &feature_per_cam = feature_per_frame[frame_id];
      const std::vector<int> obs_cam_ids = feature_per_cam.getObsCamIds();

      const FrameBundlePtr &frame_bundle = states[frame_id].frame_bundle;

      int cam_idx = obs_cam_ids.front();
      const FramePtr &frame = frame_bundle->at(cam_idx);
      const Eigen::Vector3d &pts = feature_per_cam.getObservation(cam_idx);

      Eigen::Vector3d pts_cam = pts * estimated_depth;
      Eigen::Vector3d pts_world = frame->T_world_cam() * pts_cam;
      
      setLandmarkPos(frame->landmark_vec_[ftr_idx], pts_world);
    }
  }
}

bool VinsBackendInterface::motionDetect(
    const FrameBundlePtr& last_frames, 
    const FrameBundlePtr& new_frames,
    const ImuMeasurements& imu_measurements,
    const OdomMeasurements& odom_measurements)
{
  if(!vins_estimator_)
    return false;

  return vins_estimator_->motionDetect(last_frames, new_frames, 
                          imu_measurements, odom_measurements);
}

void VinsBackendInterface::getMargedKF(FrameBundlePtr& marged_KF)
{
  vins_estimator_->getMargedKF(marged_KF);
}

Eigen::Vector3d VinsBackendInterface::getOdomVelScale()
{
  return vins_estimator_->getOdomVelScale();
}

Eigen::Vector3d VinsBackendInterface::getOdomGyrScale()
{
  return vins_estimator_->getOdomGyrScale();
}

bool VinsBackendInterface::isBackendOK()
{
  return vins_estimator_->isBackendOK();
}

const Eigen::Matrix3d VinsBackendInterface::expMatrix(const Eigen::Vector3d & omega)
{
  double theta;
  theta = omega.norm();
  double half_theta = 0.5 * theta;

  double imag_factor;
  double real_factor = std::cos(half_theta);
  if(theta < 1e-10)
  {
    double theta_sq = theta * theta;
    double theta_po4 = theta_sq * theta_sq;
    imag_factor = 0.5 - 0.0208333 * theta_sq + 0.000260417 * theta_po4;
  }
  else
  {
    double sin_half_theta = std::sin(half_theta);
    imag_factor = sin_half_theta / theta;
  }

  Eigen::Quaterniond q(real_factor, imag_factor * omega.x(),
              imag_factor * omega.y(), imag_factor * omega.z());
  q.normalize();
  
  return q.toRotationMatrix();
}

const Eigen::Matrix3d VinsBackendInterface::JacobianR(const Eigen::Vector3d& w)
{
  Eigen::Matrix3d Jr = Eigen::Matrix3d::Identity();
  double theta = w.norm();
  if(theta < 0.00001)
  {
      return Jr;// = Matrix3d::Identity();
  }
  else
  {
      Eigen::Vector3d k = w.normalized();  // k - unit direction vector of w
      Eigen::Matrix3d K;
      K <<     0, -k(2),  k(1),
            k(2),     0, -k(0),
           -k(1),  k(0),     0;
      Jr = Eigen::Matrix3d::Identity()
            - (1-cos(theta))/theta*K
            + (1-sin(theta)/theta)*K*K;
  }
  return Jr;
}

const Eigen::Matrix3d VinsBackendInterface::JacobianL(const Eigen::Vector3d& w)
{
  return JacobianR(-w);
}

void VinsBackendInterface::drawInitMonoMatches()
{
  if(!DEBUG)
    return;

  size_t cam_size = vins_estimator_->states_[0].frame_bundle->size();

  for(size_t cam_idx = 0; cam_idx < cam_size; ++cam_idx)
  {
    for(int i = 0; i < WINDOW_SIZE - 1; ++i)
    {
      FrameBundlePtr &frame_bundle_i = vins_estimator_->states_[i].frame_bundle;
      const FramePtr &frame_i = frame_bundle_i->at(cam_idx);
      cv::Mat img_i = frame_i->img_pyr_[0];
      int cols = img_i.cols;

      for(int j = i + 1; j < WINDOW_SIZE; ++j)
      {
        FrameBundlePtr &frame_bundle_j = vins_estimator_->states_[j].frame_bundle;
        const FramePtr &frame_j = frame_bundle_j->at(cam_idx);
        cv::Mat img_j = frame_j->img_pyr_[0];
        
        cv::Mat match_img;
        cv::hconcat(img_i, img_j, match_img);
        if(match_img.channels() == 1)
          cv::cvtColor(match_img, match_img, cv::COLOR_GRAY2RGB);

        cv::Mat unproject_img;
        unproject_img = match_img.clone();

        for(auto &feature_id : vins_estimator_->frame_manager_->feature_ids_)
        {
          auto &obs_frames = vins_estimator_->frame_manager_->feature_per_ids_[feature_id].obs_frames;
          auto &feature_per_frame = vins_estimator_->frame_manager_->feature_per_ids_[feature_id].feature_per_frame;

          std::vector<int>::iterator iter_i = std::find(obs_frames.begin(), obs_frames.end(), i);
          std::vector<int>::iterator iter_j = std::find(obs_frames.begin(), obs_frames.end(), j);

          if(iter_i != obs_frames.end() && iter_j != obs_frames.end())
          {
            auto &feature_per_frame_i = feature_per_frame[i];
            auto &feature_per_frame_j = feature_per_frame[j];
            bool obs_by_i = feature_per_frame_i.isObsByCam(cam_idx);
            bool obs_by_j = feature_per_frame_j.isObsByCam(cam_idx);

            if(!obs_by_i || !obs_by_j)
              continue;

            int idx_i = feature_per_frame_i.getObsFtrIdx(cam_idx);
            int idx_j = feature_per_frame_j.getObsFtrIdx(cam_idx);

            const Eigen::Vector2d &px_i = feature_per_frame_i.getObsPixel(cam_idx);
            cv::Point2d point_i(px_i.x(), px_i.y());

            const Eigen::Vector2d &px_j = feature_per_frame_j.getObsPixel(cam_idx);
            cv::Point2d point_j(px_j.x() + cols, px_j.y());

            cv::circle(match_img, point_i, 2, cv::Scalar(0, 255, 0), 2);
            cv::circle(match_img, point_j, 2, cv::Scalar(0, 255, 0), 2);
            cv::line(match_img, point_i, point_j, cv::Scalar(0, 255, 255), 1, 8, 0);

            cv::circle(unproject_img, point_i, 2, cv::Scalar(0, 255, 0), 2);
            cv::circle(unproject_img, point_j, 2, cv::Scalar(0, 255, 0), 2);

            // if(!frame_i->landmark_vec_[idx_i] || !frame_j->landmark_vec_[idx_j])
            //   continue;

            if(frame_i->landmark_vec_[idx_i])
            {
              Eigen::Vector3d pts_world_i = getLandmarkPos(frame_i->landmark_vec_[idx_i]); 
              Eigen::Vector3d pts_cam_i = frame_i->T_cam_world() * pts_world_i;
              Eigen::Vector2d px_i = Eigen::Vector2d::Zero();
              frame_i->cam()->project3(pts_cam_i, &px_i);
            
              cv::Point2d proj_point_i(px_i.x(), px_i.y());
              cv::circle(unproject_img, proj_point_i, 2, cv::Scalar(0, 0, 255), 2);

              cv::line(unproject_img, point_i, proj_point_i, cv::Scalar(0, 255, 255), 2, 8, 0);
            }

            if(frame_j->landmark_vec_[idx_j])
            {
              Eigen::Vector3d pts_world_j = getLandmarkPos(frame_j->landmark_vec_[idx_j]);
              Eigen::Vector3d pts_cam_j = frame_j->T_cam_world() * pts_world_j;
              Eigen::Vector2d px_j = Eigen::Vector2d::Zero();
              frame_j->cam()->project3(pts_cam_j, &px_j);

              cv::Point2d proj_point_j(px_j.x() + cols, px_j.y());

              cv::circle(unproject_img, proj_point_j, 2, cv::Scalar(0, 0, 255), 2);

              cv::line(unproject_img, point_j, proj_point_j, cv::Scalar(0, 255, 255), 2, 8, 0);
            }
          }
        }

        std::string home_path = getenv("HOME");

        char match_path[256];
        snprintf(match_path, sizeof(match_path), "%s/debug/mono_init_match_%d_%d.jpg", 
                  home_path.c_str(), (int)i, (int)j);
        cv::imwrite(match_path, match_img);

        char unproject_path[256];
        snprintf(unproject_path, sizeof(unproject_path), "%s/debug/mono_init_unproject_%d-%d_%d.jpg", 
                  home_path.c_str(), (int)i, (int)j, (int)cam_idx);
        cv::imwrite(unproject_path, unproject_img);
      }
    }
  }
  
}

void VinsBackendInterface::drawInitStereoMatches()
{
  if(!DEBUG)
    return;

  for(int i = 0; i <= WINDOW_SIZE; ++i)
  {
    FrameBundlePtr &frame_bundle = vins_estimator_->states_[i].frame_bundle;
    const FramePtr &frame0 = frame_bundle->at(0);
    const FramePtr &frame1 = frame_bundle->at(1);
    std::unordered_map<size_t, size_t> feature_matches;
    vins_estimator_->frame_manager_->getFeatureMatches(frame0, frame1, feature_matches);
 
    cv::Mat match_img;
    cv::Mat left_img = frame0->img_pyr_[0];
    cv::Mat right_img = frame1->img_pyr_[0];
    int cols = left_img.cols;
    cv::hconcat(left_img, right_img, match_img);

    if(match_img.channels() == 1)
      cv::cvtColor(match_img, match_img, cv::COLOR_GRAY2RGB);
    
    cv::Mat unproject_img;
    unproject_img = match_img.clone();

    for(auto &iter : feature_matches)
    {
      size_t idx0 = iter.first;
      size_t idx1 = iter.second;
      cv::Point2d left_point(frame0->px_vec_.col(idx0)[0], frame0->px_vec_.col(idx0)[1]);
      cv::Point2d right_point(frame1->px_vec_.col(idx1)[0] + cols, frame1->px_vec_.col(idx1)[1]);

      if(frame0->landmark_vec_[idx0] == nullptr || frame1->landmark_vec_[idx1] == nullptr)
        continue;

      Eigen::Vector3d pts0_world = getLandmarkPos(frame0->landmark_vec_[idx0]);
      Eigen::Vector3d pts0_cam = frame0->T_cam_world() * pts0_world;
      Eigen::Vector2d px0 = Eigen::Vector2d::Zero();
      frame0->cam()->project3(pts0_cam, &px0);
      cv::Point2d proj_left_point(px0.x(), px0.y());

      Eigen::Vector3d pts1_world = getLandmarkPos(frame1->landmark_vec_[idx1]);
      Eigen::Vector3d pts1_cam = frame1->T_cam_world() * pts1_world;
      Eigen::Vector2d px1 = Eigen::Vector2d::Zero();
      frame1->cam()->project3(pts1_cam, &px1);
      cv::Point2d proj_right_point(px1.x() + cols, px1.y());

      cv::circle(match_img, left_point, 2, cv::Scalar(0, 255, 0), 2);
      cv::circle(match_img, right_point, 2, cv::Scalar(0, 255, 0), 2);

      cv::circle(match_img, proj_left_point, 2, cv::Scalar(255, 0, 0), 2);
      cv::circle(match_img, proj_right_point, 2, cv::Scalar(0, 0, 255), 2);
      
      cv::line(match_img, left_point, proj_left_point, cv::Scalar(0, 255, 255), 1, 8, 0);
      cv::line(match_img, right_point, proj_right_point, cv::Scalar(0, 255, 255), 1, 8, 0);

      cv::circle(unproject_img, left_point, 2, cv::Scalar(0, 255, 0), 2);
      cv::circle(unproject_img, right_point, 2, cv::Scalar(0, 255, 0), 2);
      cv::line(unproject_img, left_point, right_point, cv::Scalar(0, 255, 255), 1, 8, 0);
    }

    char match_path[256];
    std::string home_path = getenv("HOME");
    snprintf(match_path, sizeof(match_path), "%s/debug/%d_init_match.jpg", home_path.c_str(), i);
    cv::imwrite(match_path, match_img);

    char unproject_path[256];
    snprintf(unproject_path, sizeof(unproject_path), "%s/debug/%d_init_unproject.jpg", home_path.c_str(), i);
    cv::imwrite(unproject_path, unproject_img);
  }
}

void VinsBackendInterface::printInitResult()
{
  for(int i = 0; i <= WINDOW_SIZE; ++i)
  {
    FrameBundlePtr &frame_bundle = vins_estimator_->states_[i].frame_bundle;
    printf("frame: %d, ts: %.9f\n", i, getTimestamp(frame_bundle));
    for(size_t j = 0; j < frame_bundle->size(); ++j)
    {
      printf("camera: %ld, landmark size: %ld; ", j, 
            numLandmarks(frame_bundle->at(j)));  
    }
    std::cout << std::endl;

    Transformation pose = getImu2WorldTrans(frame_bundle);
    std::cout << "pose: \n" << pose << std::endl;
  }
}

} // namespace svo 
