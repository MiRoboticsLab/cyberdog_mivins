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

#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "estimator/estimator.h"

Estimator::Estimator(const CameraBundlePtr& camera_bundle)
  :camera_bundle_(camera_bundle), start_thread_(false), backend_quit_(false)
{
  printf("init begins\n");
  if(!camera_bundle)
  {
    printf("camera_bundle_ is nullptr\n");
    exit(-1);
  }

  clearState();
  setParameter();

  // create debug directory
  char debug_path[256];
  std::string home_path = getenv("HOME");
  snprintf(debug_path, sizeof(debug_path), "%s/debug", home_path.c_str());
  if(access(debug_path, 0) == -1)
  {
    mkdir(debug_path, S_IRWXU);                                                                                                                                                             
    printf("debug_path: %s is not exist, it would be created!\n", debug_path);
  }
  
  char be_ts_cost_file[512];
  snprintf(be_ts_cost_file, sizeof(be_ts_cost_file), 
          "%s/debug/backend_ts_cost.txt", home_path.c_str());
  // fp_be_ts_cost_ = fopen(be_ts_cost_file, "wb");
  // save_be_ts_.open(be_ts_cost_file);
  // save_be_ts_.precision(6);
}

Estimator::Estimator(const CameraBundlePtr& camera_bundle,
  const VinsBackendOptions& options)
  :options_(options), 
  camera_bundle_(camera_bundle), 
  start_thread_(false)
{
  printf("init begins\n");
  if(!camera_bundle)
  {
    printf("camera_bundle_ is nullptr\n");
    exit(-1);
  }

  clearState();
  setParameter();

  // create debug directory
  char debug_path[256];
  std::string home_path = getenv("HOME");
  snprintf(debug_path, sizeof(debug_path), "%s/debug", home_path.c_str());
  if(access(debug_path, 0) == -1)
  {
    mkdir(debug_path, S_IRWXU);                                                                                                                                                             
    printf("debug_path: %s is not exist, it would be created!\n", debug_path);
  }

  char be_ts_cost_file[512];
  snprintf(be_ts_cost_file, sizeof(be_ts_cost_file), 
          "%s/debug/backend_ts_cost.txt", home_path.c_str());
  // fp_be_ts_cost_ = fopen(be_ts_cost_file, "wb");
  // save_be_ts_.open(be_ts_cost_file);
  // save_be_ts_.precision(6);
}


Estimator::~Estimator()
{
  backend_quit_ = true;

  if (MULTIPLE_THREAD)
  {
    start_thread_ = false;
    process_thread_.join();
    printf("join thread \n");
  }

  // save_be_ts_.close();
}

void Estimator::reset()
{
  clearState();
  setParameter();
}

void Estimator::clearState()
{
  std::lock_guard<std::mutex> lock(process_mutex_);

  while(!marg_done_)
  {
    std::chrono::milliseconds dura(5);
    std::this_thread::sleep_for(dura);
    std::cout << "Wait for marg done\n";
  }

  imu_queue_ = std::queue<ImuData>();
  odom_queue_ = std::queue<OdomData>();

  tmp_pre_integration_.reset();
  tmp_pre_integration_ = nullptr;

  tmp_odom_integration_.reset();
  tmp_odom_integration_ = nullptr;

  // states_.resize(WINDOW_SIZE + 1);
  states_ = std::vector<StateGroup>(WINDOW_SIZE + 1, StateGroup());

  tic_.clear();
  ric_.clear();
  all_image_frame_.clear();

  frame_count_ = 0;
  initial_ts_ = 0.0f;
  solver_flag_ = SolverFlag::INITIAL;

  if (last_marginalization_info_ != nullptr)
  {
    delete last_marginalization_info_;
    last_marginalization_info_ = nullptr;        
  }
  last_marginalization_parameter_blocks_.clear();

  if(frame_manager_)
    frame_manager_->clearState();

  first_imu_ = false;
  first_odom_ = false;
  failure_occur_ = false;
  init_first_pose_ = false;
  cam_ex_estimate_ = false;
  odom_ex_estimate_ = false;

  deleteDoubleArray();
}

void Estimator::setParameter()
{
  std::lock_guard<std::mutex> lock(process_mutex_);

  std::cout << "plane dist: " << options_.plane_dist << std::endl;

  if(BACKEND_OPT)
    std::cout << "backend optimize enable\n";
  else
    std::cout << "backend optimize disable\n";

  if(MULTIPLE_THREAD)
  {
    options_.marg_in_thread = false;
    std::cout << "backend multiple thread enable\n";
  }
  else
    std::cout << "backend multiple thread disable\n";

  if(ERROR_TYPE == ErrorType::kUnitPlane)
  {
    std::cout << "error type: unit plane error would be used in backend\n";
    USE_BEARING_VECTOR = false;
  }
  else if(ERROR_TYPE == ErrorType::kSpherePlane)
  {
    std::cout << "error type: sphere plane error would be used in backend\n";
    USE_BEARING_VECTOR = true;
  }
  
  grain_size_ = options_.grain_size;
  cam_size_ = camera_bundle_->getNumCameras();
  std::cout << "num of cam: " << cam_size_ << std::endl;
  
  for(int i = 0; i < cam_size_; ++i)
  {
    Transformation T_C_B = camera_bundle_->get_T_C_B(i);
    Transformation T_B_C = inverse(T_C_B); // camera_bundle_->get_T_C_B(i).inverse();

    Eigen::Vector3d t_i_c = getPosition(T_B_C);
    Eigen::Matrix3d r_i_c = getRotationMatrix(T_B_C);

    TIC.emplace_back(t_i_c);
    RIC.emplace_back(r_i_c);

    tic_.emplace_back(t_i_c);
    ric_.emplace_back(r_i_c);

    std::cout << "Exitrinsic(T_imu_cam): cam " << i << "\n"  
          << T_B_C << "\n";
  }

  int obs_threshold = options_.obs_threshold;
  frame_manager_ = std::make_shared<FrameManager>(&states_, &ric_, 
                            &tic_, camera_bundle_, options_);
  
  optimize_pose_ = std::make_shared<OptimizePose>(OptimizePose::getDefaultSolverOptions());

  initializer_ = std::make_shared<Initializer>(&states_, &g_, frame_manager_);
  // for pinhole camera model
  Eigen::VectorXd intrinsic = camera_bundle_->getCamera(0).getIntrinsicParameters();
  FOCAL_LENGTH = sqrt(intrinsic[0] * intrinsic[1]);
  std::cout << "focal length: " << FOCAL_LENGTH << std::endl;

  // VisualBaseFactor::sqrt_info_ = FOCAL_LENGTH / 1.5 * Eigen::Matrix2d::Identity();

  int pyr_level = 6;
  double scale_factor = 2.0f;
  VisualBaseFactor::setErrorInfo(pyr_level, scale_factor, FOCAL_LENGTH / 1.5);

  plane_d_ = options_.plane_dist;
  td_imu_ = TD_IMU;
  g_ = G;

  std::cout << "set td_imu_cam: " << td_imu_ << std::endl;
  std::cout << "set g: " << g_.transpose() << std::endl;
  
  frame_ts_ = -1;

  trans_org_ = Eigen::Vector3d::Zero();
  rot_org_ = Eigen::Matrix3d::Identity();

  trans_opt_ = Eigen::Vector3d::Zero();
  rot_opt_ = Eigen::Matrix3d::Identity();

  last_marged_ts_ = 0.0f;
  marged_frames_ = nullptr;

  for(int i = 0; i < WINDOW_SIZE + 1; ++i)
    prev_obs_count_.emplace_back(0);

  if(MULTIPLE_THREAD && !start_thread_)
  {
    start_thread_ = true;
    process_thread_ = std::thread(&Estimator::process, this);
  }

  createDoubleArray();

  marg_done_ = true;
}

void Estimator::createDoubleArray()
{
  para_Pose_ = new double* [(WINDOW_SIZE + 1)];
  para_SpeedBias_ = new double* [(WINDOW_SIZE + 1)];

  for (int i = 0; i < WINDOW_SIZE + 1; ++i)
  {
    para_Pose_[i] = new double[SIZE_POSE];
    para_SpeedBias_[i] = new double[SIZE_SPEEDBIAS];
  }

  para_InvDepth_ = new double* [NUM_OF_F];
  for(size_t i = 0; i < NUM_OF_F; ++i)
    para_InvDepth_[i] = new double[SIZE_DEPTH];

  para_CamEx_Pose_ = new double* [cam_size_];
  for(int i = 0; i < cam_size_; ++i)
    para_CamEx_Pose_[i] = new double[SIZE_POSE];
  
  para_OdomEx_Pose_ = new double[SIZE_POSE];

  para_Td_imu_ = new double [SIZE_TD];

  para_plane_d_ = new double [1];
}

void Estimator::deleteDoubleArray()
{
  if(para_Pose_)
  {
    delete[] para_Pose_;
    para_Pose_ = nullptr;
  } 
  
  if(para_SpeedBias_)
  {
    delete[] para_SpeedBias_;
    para_SpeedBias_ = nullptr;
  } 
  
  if(para_InvDepth_) 
  {
    delete[] para_InvDepth_;
    para_InvDepth_ = nullptr;
  }
  
  if(para_CamEx_Pose_)
  {
    delete[] para_CamEx_Pose_;
    para_CamEx_Pose_ = nullptr;
  }
  
  if(para_OdomEx_Pose_) 
  {
    delete[] para_OdomEx_Pose_;
    para_OdomEx_Pose_ = nullptr;
  }
  
  if(para_Td_imu_) 
  {
    delete[] para_Td_imu_;
    para_Td_imu_ = nullptr;
  }

  if(para_plane_d_)
  {
    delete[] para_plane_d_;
    para_plane_d_ = nullptr;
  }
}

void Estimator::setImuHandler(const std::shared_ptr<ImuHandler> imu_handler)
{
  if(!BACKEND_WITH_IMU)
    return;

  imu_handler_ = imu_handler;
  use_imu_ = imu_handler ? true : false; 

  if(imu_handler_)
  {
    ACC_N = imu_handler_->imu_calib_.acc_noise_density;
    ACC_W = imu_handler_->imu_calib_.acc_bias_random_walk_sigma;
    GYR_N = imu_handler_->imu_calib_.gyro_noise_density;
    GYR_W = imu_handler_->imu_calib_.gyro_bias_random_walk_sigma;
    G.z() = imu_handler_->imu_calib_.gravity_magnitude;
    TD_IMU = imu_handler_->imu_calib_.delay_imu_cam;
    td_imu_ = TD_IMU;
    g_ = G;
    std::cout << "Imu would be used in backend\n";
    std::cout << "ACC_N: " << ACC_N << "\n";
    std::cout << "ACC_W: " << ACC_W << "\n";
    std::cout << "GYR_N: " << GYR_N << "\n";
    std::cout << "GYR_W: " << GYR_W << "\n";
    std::cout << "G norm: " << G.z() << "\n";
  }
  else
  {
    std::cout << "Imu would not be used in backend\n";        
    ESTIMATE_CAM_EXTRINSIC = false;
    ESTIMATE_IMU_TD = false;
    printf("no imu, fix extrinsic param; no time offset calibration\n");
  }
}

void Estimator::setOdomHandler(const std::shared_ptr<OdomHandler> odom_handler)
{
  if(!BACKEND_WITH_ODOM)
    return;

  odom_handler_ = odom_handler;
  use_odom_ = odom_handler ? true : false;

  if(odom_handler_)
  {
    Transformation &T_B_O = odom_handler->odom_calib_.T_B_O;
    TIO = getPosition(T_B_O);
    RIO = getRotationMatrix(T_B_O); 
    

    TD_ODOM = odom_handler->odom_calib_.delay_odom;
    td_odom_ = TD_ODOM;
    rio_ = RIO;
    tio_ = TIO;
    roi_ = rio_.transpose();
    toi_ = -rio_.transpose() * tio_;

    odom_vel_scale_ = odom_handler->odom_calib_.vel_scale;
    odom_gyr_scale_ = odom_handler->odom_calib_.gyr_scale;

    ODOM_VEL_N = odom_handler->odom_calib_.linear_velocity_n;
    ODOM_GYR_N = odom_handler->odom_calib_.angular_velocity_n;

    std::cout << "Odom vel scale: " << odom_vel_scale_.transpose() << "\n";
    std::cout << "Odom gyr scale: " << odom_gyr_scale_.transpose() << "\n";

    std::cout << "Odom would be used in backend\n";
  }
  else
    std::cout << "Odom would not be used in backend\n";        
}

void Estimator::inputImu(const double ts, 
  const Eigen::Vector3d &imu_acc, 
  const Eigen::Vector3d &imu_gyr)
{
  if(!BACKEND_WITH_IMU)
    return;

  std::lock_guard<std::mutex> lock(buf_mutex_);
  ImuData imu_data(ts, imu_acc, imu_gyr);
  imu_queue_.push(imu_data);
  //printf("input imu with time %f \n", ts);
}

void Estimator::inputImu(const ImuData &imu_data)
{
  if(!BACKEND_WITH_IMU)
    return;

  std::lock_guard<std::mutex> lock(buf_mutex_);
  imu_queue_.push(imu_data);
  //printf("input imu with time %f \n", imu_data.timestamp);
}

void Estimator::inputOdom(const OdomData &odom_data)
{
  if(!BACKEND_WITH_ODOM)
    return;

  std::lock_guard<std::mutex> lock(buf_mutex_);
  odom_queue_.push(odom_data);
  //printf("input odom with time %f \n", odom_data.timestamp);
}


void Estimator::inputOdom(const double ts,
  const Eigen::Quaterniond &odom_rot, const Eigen::Vector3d &odom_pos,
  const Eigen::Vector3d &odom_vel, const Eigen::Vector3d &odom_gyr)
{
  if(!BACKEND_WITH_ODOM)
    return;

  std::lock_guard<std::mutex> lock(buf_mutex_);
  OdomData odom_data(ts, odom_rot, odom_pos, odom_vel, odom_gyr);
  odom_queue_.push(odom_data);
  //printf("input odom with time %f \n", ts);
}    

bool Estimator::getImuMeasurements(const double t0, const double t1, std::vector<ImuData> &imu_vec)
{
  if(imu_queue_.empty())
  {
    printf("not receive imu\n");
    return false;
  }

  // printf("get imu from %f %f\n", t0, t1);
  // printf("imu fornt time %f; imu end time %f\n", 
  //     imu_queue_.front().timestamp, imu_queue_.back().timestamp);

  const double td_imu0 = frame_count_ > 0 ? states_[frame_count_ - 1].td_imu : 0.0f;
  const double td_imu1 = states_[frame_count_].td_imu;

  std::lock_guard<std::mutex> lock(buf_mutex_);

  if(t1 + td_imu1 <= imu_queue_.back().timestamp)
  {
    while (!imu_queue_.empty() && imu_queue_.front().timestamp <= t0 + td_imu0)
      imu_queue_.pop();

    while (!imu_queue_.empty() && imu_queue_.front().timestamp < t1 + td_imu1)
    {
      imu_vec.push_back(imu_queue_.front());
      imu_queue_.pop();
    }

    if(!imu_queue_.empty())
      imu_vec.push_back(imu_queue_.front());
  }
  else
  {
    printf("wait for imu\n");
    return false;
  }
  return true;
}

bool Estimator::getOdomMeasurements(const double t0, const double t1, std::vector<OdomData> &odom_vec)
{
  if(odom_queue_.empty())
  {
    printf("not receive odom\n");
    return false;
  }
  
  double td_odom0 = frame_count_ > 0 ? states_[frame_count_ - 1].td_odom : 0.0f;
  double td_odom1 = states_[frame_count_].td_odom;

  std::lock_guard<std::mutex> lock(buf_mutex_);

  if(t1 + td_odom1 <= odom_queue_.back().timestamp)
  {
    while (!odom_queue_.empty() && odom_queue_.front().timestamp < t0 + td_odom0)
      odom_queue_.pop();

    while (!odom_queue_.empty() && odom_queue_.front().timestamp < t1 + td_odom1)
    {
      odom_vec.push_back(odom_queue_.front());
      odom_queue_.pop();
    }

    if(!odom_queue_.empty())
      odom_vec.push_back(odom_queue_.front());
  }
  else
  {
    printf("wait for odom\n");
    return false;
  }

  return true;
}

bool Estimator::imuAvailable(const double ts)
{
  const double &td_imu = states_[frame_count_].td_imu;
  if(!imu_queue_.empty() && ts + td_imu <= imu_queue_.back().timestamp)
    return true;

  return false;
}

bool Estimator::odomAvailable(const double ts)
{
  const double &td_odom = states_[frame_count_].td_odom;
  if(!odom_queue_.empty() && ts + td_odom <= odom_queue_.back().timestamp)
    return true;

  return false;
}

void Estimator::interpolationImu(const int i, const double cur_ts, std::vector<ImuData> &imu_vec)
{
  double dt_1 = cur_ts - imu_vec[i - 1].timestamp;
  double dt_2 = imu_vec[i].timestamp - cur_ts;
  assert(dt_1 >= 0.0f);
  assert(dt_2 >= 0.0f);
  // printf("dt_1: %.9f, dt_2: %.9f\n", dt_1, dt_2);

  double w1 = dt_2 / (dt_1 + dt_2);
  double w2 = dt_1 / (dt_1 + dt_2);

  double a_x = w1 * imu_vec[i - 1].linear_acceleration.x() + w2 * imu_vec[i].linear_acceleration.x();
  double a_y = w1 * imu_vec[i - 1].linear_acceleration.y() + w2 * imu_vec[i].linear_acceleration.y();
  double a_z = w1 * imu_vec[i - 1].linear_acceleration.z() + w2 * imu_vec[i].linear_acceleration.z();
  double g_x = w1 * imu_vec[i - 1].angular_velocity.x() + w2 * imu_vec[i].angular_velocity.x();
  double g_y = w1 * imu_vec[i - 1].angular_velocity.y() + w2 * imu_vec[i].angular_velocity.y();
  double g_z = w1 * imu_vec[i - 1].angular_velocity.z() + w2 * imu_vec[i].angular_velocity.z();
  // std::cout << "org: " << imu_vec[i].linear_acceleration.transpose() << ", " 
  //                      << imu_vec[i].angular_velocity.transpose() << "\n";
  imu_vec[i].linear_acceleration = Eigen::Vector3d(a_x, a_y, a_z);
  imu_vec[i].angular_velocity = Eigen::Vector3d(g_x, g_y, g_z);
  // std::cout << "new: " << imu_vec[i].linear_acceleration.transpose() << ", " 
  //                      << imu_vec[i].angular_velocity.transpose() << "\n";
}

void Estimator::interpolationOdom(const int i, const double cur_ts, std::vector<OdomData> &odom_vec)
{
  Eigen::Vector3d linear_velocity = odom_vec[i].linear_velocity;
  Eigen::Vector3d angular_velocity = odom_vec[i].angular_velocity;
  Eigen::Vector3d position = odom_vec[i].position;
  Eigen::Quaterniond orientation = odom_vec[i].orientation;

  valueInterpolation(odom_vec[i - 1].timestamp, odom_vec[i - 1].linear_velocity,
                    odom_vec[i].timestamp, odom_vec[i].linear_velocity,
                    cur_ts, linear_velocity);

  valueInterpolation(odom_vec[i - 1].timestamp, odom_vec[i - 1].angular_velocity,
                    odom_vec[i].timestamp, odom_vec[i].angular_velocity,
                    cur_ts, angular_velocity);

  poseInterpolation(odom_vec[i - 1].timestamp, odom_vec[i - 1].orientation, odom_vec[i - 1].position,
                    odom_vec[i].timestamp, odom_vec[i].orientation, odom_vec[i].position,
                    cur_ts, orientation, position);

  // std::cout << "interpolationOdom: "
  //           << "i - 1: " << odom_vec[i - 1].position.transpose() << "; "
  //           << "i: " << odom_vec[i].position.transpose() << "; "
  //           << "res: " << position.transpose() << "\n";

  odom_vec[i].position = position;
  odom_vec[i].orientation = orientation;
  odom_vec[i].linear_velocity = linear_velocity;
  odom_vec[i].angular_velocity = angular_velocity;
}

void Estimator::initFirstImuPose(std::vector<ImuData> &imu_vec)
{
  printf("init first imu pose\n");
  init_first_pose_ = true;
  //return;
  Eigen::Vector3d aver_acc(0, 0, 0);
  int n = (int)imu_vec.size();
  for(size_t i = 0; i < imu_vec.size(); ++i)
  {
    aver_acc += imu_vec[i].linear_acceleration;
  }
  aver_acc = aver_acc / n;
  printf("averge acc %f %f %f\n", aver_acc.x(), aver_acc.y(), aver_acc.z());
  Eigen::Matrix3d R0 = Utility::g2R(aver_acc);
  double yaw = Utility::R2ypr(R0).x();
  R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
  states_[0].R = R0;
  std::cout << "init R0 " << std::endl << states_[0].R << std::endl;
}

void Estimator::processImu(const double prv_ts, const double cur_ts,
    std::vector<ImuData> imu_vec, Eigen::Matrix3d &R, 
    Eigen::Vector3d &P, Eigen::Vector3d &V,
    Eigen::Vector3d &Ba, Eigen::Vector3d &Bg)
{
  for(size_t i = 0; i < imu_vec.size(); ++i)
  {
    double dt = 0.0f;
    double imu_ts = imu_vec[i].timestamp;

    if(i == 0)
      dt = imu_ts - prv_ts;
    else if (i == imu_vec.size() - 1)
    {
      dt = cur_ts - imu_vec[i - 1].timestamp;
      interpolationImu(i, cur_ts, imu_vec);
    }
    else
      dt = imu_ts - imu_vec[i - 1].timestamp;

    const Eigen::Vector3d &acc = imu_vec[i].linear_acceleration;
    const Eigen::Vector3d &gyr = imu_vec[i].angular_velocity;

    if (!first_imu_)
    {
      first_imu_ = true;
      acc_prv_ = acc;
      gyr_prv_ = gyr;
    }

    if (!states_[frame_count_].pre_integration)
    {
      states_[frame_count_].pre_integration = std::make_shared<IntegrationBase>(
          acc_prv_, gyr_prv_, Ba, Bg);
    }

    if (frame_count_ != 0)
    {
      states_[frame_count_].pre_integration->pushBack(dt, acc, gyr, propagate_imu_);
      if(solver_flag_ == SolverFlag::INITIAL && tmp_pre_integration_)
        tmp_pre_integration_->pushBack(dt, acc, gyr, propagate_imu_);

      states_[frame_count_].imu_dt_buf.push_back(dt);
      states_[frame_count_].imu_acc_buf.push_back(acc);
      states_[frame_count_].imu_gyr_buf.push_back(gyr);

      Eigen::Matrix3d prv_R = R;
      Eigen::Vector3d un_gyr = 0.5 * (gyr_prv_ + gyr) - Bg;
      R *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();

      Eigen::Vector3d un_acc_0 = prv_R * (acc_prv_ - Ba) - g_;
      Eigen::Vector3d un_acc_1 = R * (acc - Ba) - g_;
      Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

      P += dt * V + 0.5 * dt * dt * un_acc;
      V += dt * un_acc;
    }

    acc_prv_ = acc;
    gyr_prv_ = gyr; 
  }
}

void Estimator::processOdom(const double prv_ts, const double cur_ts, 
    std::vector<OdomData> &odom_vec, Eigen::Matrix3d &R, Eigen::Vector3d &P)
{
  for(size_t i = 0; i < odom_vec.size(); ++i)
  {
    double dt = 0.0f;
    double odom_ts = odom_vec[i].timestamp;
    // printf("%.9lf %.6lf %.6lf %.6lf %.6lf %.6lf %.6lf\n",
    //        odom_ts, odom_vec[i].linear_velocity.x(), odom_vel_vec[i].linear_velocity.y(), 
    //           odom_vel_vec[i].linear_velocity.z(), odom_gyr_vec[i].angular_velocity.x(), 
    //           odom_gyr_vec[i].angular_velocity.y(), odom_gyr_vec[i].angular_velocity.z());

    if(i == 0)
      dt = odom_ts - prv_ts;
    else if (i == odom_vec.size() - 1)
    {
      dt = cur_ts - odom_vec[i - 1].timestamp;
      interpolationOdom(i, cur_ts, odom_vec);
    }
    else
      dt = odom_ts - odom_vec[i - 1].timestamp;

    const OdomData &odom_data_cur = odom_vec[i];

    if (!first_odom_)
    {
      odom_data_prv_ = odom_data_cur;
      first_odom_ = true;
    }

    if (!states_[frame_count_].odom_integration)
    {
      states_[frame_count_].odom_integration = std::make_shared<IntegrationOdom>(
        odom_data_prv_.orientation, odom_data_prv_.position, odom_data_prv_.linear_velocity, 
        odom_data_prv_.angular_velocity, odom_vel_scale_, odom_gyr_scale_, td_odom_, ESTIMATE_ODOM_S3D);
      setOdomWeight(states_[frame_count_].odom_integration, states_[frame_count_].tracked_feat_cnt);
    }

    if (frame_count_ != 0)
    {
      states_[frame_count_].odom_integration->pushBack(dt, odom_data_cur.orientation, odom_data_cur.position, 
                                              odom_data_cur.linear_velocity, odom_data_cur.angular_velocity);
      if(tmp_odom_integration_)
        tmp_odom_integration_->pushBack(dt, odom_data_cur.orientation, odom_data_cur.position, 
                                odom_data_cur.linear_velocity, odom_data_cur.angular_velocity);

      states_[frame_count_].odom_dt_buf.push_back(dt);
      states_[frame_count_].odom_vel_buf.push_back(odom_data_cur.linear_velocity);
      states_[frame_count_].odom_gyr_buf.push_back(odom_data_cur.angular_velocity);
      states_[frame_count_].odom_pos_buf.push_back(odom_data_cur.position);
      states_[frame_count_].odom_rot_buf.push_back(odom_data_cur.orientation);

      int j = frame_count_;

      Eigen::Matrix3d vs = odom_vel_scale_.asDiagonal();
      Eigen::Matrix3d gs = odom_gyr_scale_.asDiagonal();

      Eigen::Matrix3d R_w_o = R * rio_;
      Eigen::Vector3d t_w_o = R * tio_ + P;

      Eigen::Vector3d odom_vel_0 = R_w_o * vs * odom_data_prv_.linear_velocity;
      Eigen::Vector3d un_odom_gyr = 0.5 * gs * (odom_data_prv_.angular_velocity 
                                    + odom_data_cur.angular_velocity);
      Eigen::Vector3d delta_theta = un_odom_gyr * dt;
      R_w_o *= Utility::deltaQ(delta_theta).normalized().toRotationMatrix();
      
      Eigen::Vector3d odom_vel_1 = R_w_o * vs * odom_data_cur.linear_velocity;
      t_w_o += 0.5 * (odom_vel_0 + odom_vel_1) * dt;

      R = R_w_o * roi_;
      P = R_w_o * toi_ + t_w_o;
    }

    odom_data_prv_ = odom_data_cur;
  }

}

void Estimator::setOdomWeight(const IntegrationOdomPtr &integration_odom, const int tracking_cnt)
{
  float weight = 1.0;

  if(tracking_cnt < options_.quality_min_fts)
    weight /= 5.0;

  integration_odom->setWeight(weight);
}

void Estimator::processImuAndOdom(const FrameBundlePtr &new_frames)
{
  const double& prv_ts = prv_ts_;
  const double& cur_ts = cur_ts_;

  std::vector<ImuData> imu_vec;
  std::vector<OdomData> odom_vec;
  
  while(!backend_quit_)
  {
    if ((!use_imu_ || imuAvailable(cur_ts)))
      break;
    else
    {
      if(!imu_queue_.empty())
      {
        double front_ts = imu_queue_.front().timestamp;
        double back_ts = imu_queue_.back().timestamp;
        printf("Imu fornt ts: %f; Imu end ts: %f\n", front_ts, back_ts);
        printf("Cam ts: %f, td: %f\n", cur_ts, states_[frame_count_].td_imu);
      }
      else 
        printf("Imu queue is empty, please check imu sensor!\n");

      if (! MULTIPLE_THREAD)
        return;
      std::chrono::milliseconds dura(5);
      std::this_thread::sleep_for(dura);
    }
  }

  while(!backend_quit_)
  {
    if ((!use_odom_  || odomAvailable(cur_ts)))
      break;
    else
    {
      if(!odom_queue_.empty())
      {
        double front_ts = odom_queue_.front().timestamp;
        double back_ts = odom_queue_.back().timestamp;
        printf("Odom fornt ts: %f; Odom end ts: %f\n", front_ts, back_ts);
      }
      else
        printf("odom queue is empty, please check odom sensor!\n");

      if (! MULTIPLE_THREAD)
        return;
      std::chrono::milliseconds dura(5);
      std::this_thread::sleep_for(dura);
    }
  }

  if(use_imu_)
    getImuMeasurements(prv_ts, cur_ts, imu_vec);
  if(use_odom_)
    getOdomMeasurements(prv_ts, cur_ts, odom_vec);

  Eigen::Matrix3d R_imu, R_odom;
  Eigen::Vector3d P_imu, P_odom;
  Eigen::Vector3d V_imu, Ba_imu, Bg_imu;
  
  /*
  Eigen::Matrix3d R_org = states_[frame_count_].R;
  Eigen::Vector3d P_org = states_[frame_count_].P;

  propagate_imu_ = !(use_imu_ && use_odom_);
  */

  if(use_imu_)
  {
    if(imu_vec.empty())
      std::cout << "imu_vec is empty\n";

    if(!init_first_pose_)
    {
      if(PIPLINE_TYPE == PipType::kMono || PIPLINE_TYPE == PipType::kRgbd)
        initFirstImuPose(imu_vec);
      else
      {
        Transformation T = getImu2WorldTrans(new_frames);
        states_[frame_count_].R = getRotationMatrix(T);
        states_[frame_count_].P = getPosition(T);
        states_[frame_count_].V = Eigen::Vector3d::Zero();
      }

      init_first_pose_ = true;
    }

    R_imu = states_[frame_count_].R;
    P_imu = states_[frame_count_].P;
    V_imu = states_[frame_count_].V;
    Ba_imu = states_[frame_count_].Ba;
    Bg_imu = states_[frame_count_].Bg;

    double prv_td_imu = frame_count_ > 0 ? states_[frame_count_ - 1].td_imu : 0.0f;
    double cur_td_imu = states_[frame_count_].td_imu;

    processImu(prv_ts + prv_td_imu, cur_ts + cur_td_imu, 
               imu_vec, R_imu, P_imu, V_imu, Ba_imu, Bg_imu);
  }

  if(use_odom_)
  {
    if(odom_vec.empty())
      std::cout << "odom_vec is empty\n";

    R_odom = states_[frame_count_].R;
    P_odom = states_[frame_count_].P;

    processOdom(prv_ts, cur_ts, odom_vec, R_odom, P_odom);
  }

  {
    if(use_imu_)
    {
      states_[frame_count_].R = R_imu;
      states_[frame_count_].P = P_imu;
      states_[frame_count_].V = V_imu;
    }
    else if(use_odom_)
    {
      states_[frame_count_].R = R_odom;
      states_[frame_count_].P = P_odom;
    }
  }

  if(is_kf_ && (use_imu_ || use_odom_))
  {
    if(use_imu_) 
      std::cout << "processIMU result: ";
    else if(use_odom_) 
      std::cout << "processOdom result: ";

    std::cout << "P " << states_[frame_count_].P.transpose() << "\n";

    if(use_imu_)
      std::cout << "Imu V: " << states_[frame_count_].V.transpose() << "; "
                << "Imu Ba: " << states_[frame_count_].Ba.transpose() << "; "
                << "Imu Bg: " << states_[frame_count_].Bg.transpose() << "\n";
  }
}

void Estimator::vector2double(const bool opt)
{
  for (int i = 0; i <= WINDOW_SIZE; ++i)
  {
    para_Pose_[i][0] = states_[i].P.x();
    para_Pose_[i][1] = states_[i].P.y();
    para_Pose_[i][2] = states_[i].P.z();
    
    Eigen::Quaterniond q{states_[i].R};
    para_Pose_[i][3] = q.x();
    para_Pose_[i][4] = q.y();
    para_Pose_[i][5] = q.z();
    para_Pose_[i][6] = q.w();

    if(use_imu_)
    {
      para_SpeedBias_[i][0] = states_[i].V.x();
      para_SpeedBias_[i][1] = states_[i].V.y();
      para_SpeedBias_[i][2] = states_[i].V.z();

      para_SpeedBias_[i][3] = states_[i].Ba.x();
      para_SpeedBias_[i][4] = states_[i].Ba.y();
      para_SpeedBias_[i][5] = states_[i].Ba.z();

      para_SpeedBias_[i][6] = states_[i].Bg.x();
      para_SpeedBias_[i][7] = states_[i].Bg.y();
      para_SpeedBias_[i][8] = states_[i].Bg.z();
    }
  }
  
  for (int i = 0; i < cam_size_; ++i)
  {
    para_CamEx_Pose_[i][0] = tic_[i].x();
    para_CamEx_Pose_[i][1] = tic_[i].y();
    para_CamEx_Pose_[i][2] = tic_[i].z();

    Eigen::Quaterniond q{ric_[i]};
    para_CamEx_Pose_[i][3] = q.x();
    para_CamEx_Pose_[i][4] = q.y();
    para_CamEx_Pose_[i][5] = q.z();
    para_CamEx_Pose_[i][6] = q.w();
  }

  if(use_odom_)
  {
    para_OdomEx_Pose_[0] = tio_.x();
    para_OdomEx_Pose_[1] = tio_.y();
    para_OdomEx_Pose_[2] = tio_.z();

    Eigen::Quaterniond q{rio_};
    para_OdomEx_Pose_[3] = q.x();
    para_OdomEx_Pose_[4] = q.y();
    para_OdomEx_Pose_[5] = q.z();
    para_OdomEx_Pose_[6] = q.w();
  }

  frame_manager_->getDepth(frame_count_, solver_flag_, opt, 
      opt_feature_ids_, para_feature_map_, para_InvDepth_);

  if(use_imu_)
    para_Td_imu_[0] = td_imu_;

  if(options_.backend_with_plane)
    para_plane_d_[0] = plane_d_;
}

void Estimator::double2vector()
{
  Eigen::Vector3d origin_R0 = Utility::R2ypr(states_[0].R);
  Eigen::Vector3d origin_P0 = states_[0].P;

  if (failure_occur_)
  {
    origin_R0 = Utility::R2ypr(last_R0_);
    origin_P0 = last_P0_;
    failure_occur_ = false;
  }

  if(use_imu_)
  {
    Eigen::Vector3d origin_R00 = Utility::R2ypr(Eigen::Quaterniond(
                    para_Pose_[0][6], para_Pose_[0][3],
                    para_Pose_[0][4], para_Pose_[0][5]
                    ).toRotationMatrix());
    double y_diff = origin_R0.x() - origin_R00.x();
    //TODO
    Eigen::Matrix3d rot_diff = Utility::ypr2R(Eigen::Vector3d(y_diff, 0, 0));
    if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
    {
      printf("euler singular point!\n");
      rot_diff = states_[0].R * Eigen::Quaterniond(
                para_Pose_[0][6], para_Pose_[0][3],
                para_Pose_[0][4], para_Pose_[0][5]
                ).toRotationMatrix().transpose();
    }

    for (int i = 0; i <= WINDOW_SIZE; ++i)
    {

      states_[i].R = rot_diff * Eigen::Quaterniond(
                para_Pose_[i][6], para_Pose_[i][3], 
                para_Pose_[i][4], para_Pose_[i][5]
                ).normalized().toRotationMatrix();
      
      states_[i].P = rot_diff * Eigen::Vector3d(
                para_Pose_[i][0] - para_Pose_[0][0],
                para_Pose_[i][1] - para_Pose_[0][1],
                para_Pose_[i][2] - para_Pose_[0][2]) + origin_P0;


      states_[i].V = rot_diff * Eigen::Vector3d(para_SpeedBias_[i][0],
                                                para_SpeedBias_[i][1],
                                                para_SpeedBias_[i][2]);

      states_[i].Ba = Eigen::Vector3d(para_SpeedBias_[i][3],
                                      para_SpeedBias_[i][4],
                                      para_SpeedBias_[i][5]);

      states_[i].Bg = Eigen::Vector3d(para_SpeedBias_[i][6],
                                      para_SpeedBias_[i][7],
                                      para_SpeedBias_[i][8]);
    }
  }
  else
  {
    for (int i = 0; i <= WINDOW_SIZE; ++i)
    {
      states_[i].R = Eigen::Quaterniond(para_Pose_[i][6], para_Pose_[i][3], 
                                        para_Pose_[i][4], para_Pose_[i][5]
                                        ).normalized().toRotationMatrix();
      
      states_[i].P = Eigen::Vector3d(para_Pose_[i][0], para_Pose_[i][1], para_Pose_[i][2]);
    }
  }

  if(use_imu_)
  {
    for (int i = 0; i < cam_size_; ++i)
    {
      tic_[i] = Eigen::Vector3d(para_CamEx_Pose_[i][0],
                                para_CamEx_Pose_[i][1],
                                para_CamEx_Pose_[i][2]);
      ric_[i] = Eigen::Quaterniond(
                  para_CamEx_Pose_[i][6], para_CamEx_Pose_[i][3],
                  para_CamEx_Pose_[i][4], para_CamEx_Pose_[i][5]
                  ).normalized().toRotationMatrix();
    }

    td_imu_ = para_Td_imu_[0];
  }

  if(use_odom_)
  {
    tio_ = Eigen::Vector3d(para_OdomEx_Pose_[0], 
                           para_OdomEx_Pose_[1],
                           para_OdomEx_Pose_[2]);

    rio_ = Eigen::Quaterniond(
              para_OdomEx_Pose_[6], para_OdomEx_Pose_[3],
              para_OdomEx_Pose_[4], para_OdomEx_Pose_[5]
              ).normalized().toRotationMatrix();

    roi_ = rio_.transpose();
    toi_ = -rio_.transpose() * tio_;
  }

  if(options_.backend_with_plane)
    plane_d_ = para_plane_d_[0];
  
  frame_manager_->setDepth(frame_count_, opt_feature_ids_, 
                      para_feature_map_, para_InvDepth_);
}

bool Estimator::failureDetection()
{
  if(!options_.use_loose_couple)
    return false;

  scope_color(ANSI_COLOR_RED_BOLD);
  // if(frame_manager_->last_track_num_ < 2)
  // {
  //   printf(" little feature %d\n", frame_manager_->last_track_num_);
  // }
  if (states_[WINDOW_SIZE].Ba.norm() > 2.5)
  {
    printf(" big IMU acc bias estimation %f\n", states_[WINDOW_SIZE].Ba.norm());
    return true;
  }
  if (states_[WINDOW_SIZE].Bg.norm() > 1.0)
  {
    printf(" big IMU gyr bias estimation %f\n", states_[WINDOW_SIZE].Bg.norm());
    return true;
  }
  /*
  if (tic_(0) > 1)
  {
    printf(" big extri param estimation %d \n", tic_(0) > 1);
    return true;
  }
  */
  Eigen::Vector3d tmp_P = states_[WINDOW_SIZE].P;
  double delta_trans = (tmp_P - last_P_).norm();
  if (delta_trans > 5)
  {
    std::cout << "tmp_P: " << tmp_P.transpose() << std::endl;
    std::cout << "last_P: " << last_P_.transpose() << std::endl;
    printf("Big translation: %.9lf\n", delta_trans);
    return true;
  }

  double delta_z = abs(tmp_P.z() - last_P_.z());
  if (delta_z > 1)
  {
    std::cout << "tmp_P.z: " << tmp_P.z() << std::endl;
    std::cout << "last_P.z: " << last_P_.z() << std::endl;
    printf("Big z translation: %.9lf\n", delta_z);
    return true; 
  }
  Eigen::Matrix3d tmp_R = states_[WINDOW_SIZE].R;
  Eigen::Matrix3d delta_R = tmp_R.transpose() * last_R_;
  Eigen::Quaterniond delta_Q(delta_R);
  double delta_angle = acos(delta_Q.w()) * 2.0 / M_PI * 180.0;
  if (delta_angle > 50)
  {
    std::cout << "tmp_R: \n" << tmp_R << std::endl;
    std::cout << "last_R: \n" << last_R_ << std::endl;
    printf("Big delta_angle: %.9lf \n", delta_angle);
    return true;
  }

  return false;
}

void Estimator::reTriangulate()
{
  if(options_.opt_kf_only)
    return;
  frame_manager_->reTriangulate(opt_feature_ids_);
}

void Estimator::initPoseByPnP()
{
  if(states_[frame_count_].tracked_feat_cnt 
      > options_.quality_min_fts)
    return;

  if(use_imu_ || use_odom_)
    return;
  
  Eigen::Vector3d t_w_b = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_w_b = Eigen::Matrix3d::Identity();
  bool res_pnp = frame_manager_->initFramePoseByPnP(
          frame_count_, R_w_b, t_w_b);
  if(res_pnp)
  {
    states_[frame_count_].R = R_w_b;
    states_[frame_count_].P = t_w_b;
  }
  else
    printf("initFramePoseByPnP failed!\n");
}

void Estimator::optimization()
{
  vector2double(true);
 
  ceres::Problem problem;
  ceres::LossFunction *loss_function;
  loss_function = new ceres::HuberLoss(1.0); // ORG
  // loss_function = new ceres::CauchyLoss(1.0);
  // loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
  // ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
 
  for(auto &feature_id : opt_feature_ids_)
  {
    const size_t &feature_index = para_feature_map_[feature_id];
    problem.AddParameterBlock(para_InvDepth_[feature_index], SIZE_DEPTH);
    // if(solver_flag_ == SolverFlag::NON_LINEAR && marg_flag_ == MargFlag::MARG_FIRST_NEW)
    // {
    //   auto &obs_frames = frame_manager_->feature_per_ids_[feature_id].obs_frames;
    //   if(obs_frames.size() > frame_manager_->obs_thresh_)
    //     problem.SetParameterBlockConstant(para_InvDepth_[feature_index]);
    // }
  }
  
  for (int i = 0; i < frame_count_ + 1; ++i)
  {
    ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
    problem.AddParameterBlock(para_Pose_[i], SIZE_POSE, local_parameterization);

    if(use_imu_)
      problem.AddParameterBlock(para_SpeedBias_[i], SIZE_SPEEDBIAS);
  }
  
  if(!use_imu_)
    problem.SetParameterBlockConstant(para_Pose_[0]);

  if(solver_flag_ == SolverFlag::NON_LINEAR && marg_flag_ == MargFlag::MARG_FIRST_NEW)
  {
    for (int i = 0; i < frame_count_; ++i)
    {
      problem.SetParameterBlockConstant(para_Pose_[i]);

      if(use_imu_)
        problem.SetParameterBlockConstant(para_SpeedBias_[i]);
    }
  }

  for (int i = 0; i < cam_size_; ++i)
  {
    ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
    problem.AddParameterBlock(para_CamEx_Pose_[i], SIZE_POSE, local_parameterization);
    if ((ESTIMATE_CAM_EXTRINSIC && frame_count_ == WINDOW_SIZE 
          && states_[0].V.norm() > 0.2) || cam_ex_estimate_)
    {
      // printf("estimate extinsic param\n");
      cam_ex_estimate_ = 1;
    }
    else
    {
      // printf("fix extinsic param\n");
      problem.SetParameterBlockConstant(para_CamEx_Pose_[i]);
    }
  }

  problem.AddParameterBlock(para_Td_imu_, 1);
  if (!ESTIMATE_IMU_TD || states_[0].V.norm() < 0.2)
    problem.SetParameterBlockConstant(para_Td_imu_);

  if(use_odom_)
  {
    ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
    problem.AddParameterBlock(para_OdomEx_Pose_, SIZE_POSE, local_parameterization);
    if(!ESTIMATE_ODOM_EXTRINSIC)
      problem.SetParameterBlockConstant(para_OdomEx_Pose_);
  }

  if (last_marginalization_info_ && last_marginalization_info_->valid)
  {
    // construct new marginlization_factor
    MarginalizationFactor *marginalization_factor = 
                  new MarginalizationFactor(last_marginalization_info_);
    problem.AddResidualBlock(marginalization_factor, NULL,
                 last_marginalization_parameter_blocks_);
  }

  if(use_imu_ && marg_flag_ == MargFlag::MARG_OLD && frame_count_ == WINDOW_SIZE)
  {
    for (int i = 0; i < frame_count_; ++i)
    {
      int j = i + 1;
      if(states_[j].pre_integration == nullptr)
        continue;

      if (states_[j].pre_integration->sum_dt_ > 10.0
        || states_[j].pre_integration->sum_dt_ == 0.0f)
      {
        printf("optimization: %d pre_integrations sum time is %.6lf\n", 
          j, states_[j].pre_integration->sum_dt_);
        continue;
      }

      IMUFactor* imu_factor = new IMUFactor(states_[j].pre_integration);
      problem.AddResidualBlock(imu_factor, NULL, para_Pose_[i], 
        para_SpeedBias_[i], para_Pose_[j], para_SpeedBias_[j]);
    }
  }

  if(use_odom_ && frame_count_ == WINDOW_SIZE) // && marg_flag_ == MargFlag::MARG_OLD
  {
    for(int i = 0; i < frame_count_; ++i)
    {
      int j = i + 1;
      if(states_[j].odom_integration == nullptr)
        continue;

      if (states_[j].odom_integration->sum_dt_ > 10.0 
          || states_[j].odom_integration->sum_dt_ == 0.0f)
      {
        printf("optimization: %d odom_integration sum time is %.6lf\n", 
          j, states_[j].odom_integration->sum_dt_);
        continue;
      }

      OdomFactor* odom_factor = new OdomFactor(states_[j].odom_integration);
      problem.AddResidualBlock(odom_factor, NULL, 
          para_Pose_[i], para_Pose_[j], para_OdomEx_Pose_);
    }
  }

  addVisualResidualForOptimize(problem, loss_function);

  double max_solver_time = options_.max_solver_time;
  if (marg_flag_ == MargFlag::MARG_OLD)
    max_solver_time *= 4.0 / 5.0;
  
  int max_num_iterations = options_.max_num_iterations;

  double solve_ts = 0.0f;
  ceres::Solver::Summary summary;
#ifdef __CPU__
  ceres::Solver::Options options;

  options.linear_solver_type = ceres::DENSE_SCHUR;
  // #ifdef USE_OPENMP
  // options.num_threads = 4;
  // #endif
  options.trust_region_strategy_type = ceres::DOGLEG;
  // options.use_explicit_schur_complement = true;
  // options.minimizer_progress_to_stdout = true;
  // options.use_nonmonotonic_steps = true;
  options.max_solver_time_in_seconds = max_solver_time;
  options.max_num_iterations = max_num_iterations;
  
  ceres::Solve(options, &problem, &summary);
  if(is_kf_)
  {
    std::cout << summary.BriefReport() << std::endl;
    printf("Iterations : %d\n", static_cast<int>(summary.iterations.size()));
  }
#endif

#ifdef __GPU__
  ceres::Solver::Options options;
  // options.num_threads = 2;
  // options.use_explicit_schur_complement = true;
  // options.minimizer_progress_to_stdout = true;
  // options.use_nonmonotonic_steps = true;

  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.trust_region_strategy_type = ceres::DOGLEG;
  options.dense_linear_algebra_library_type = ceres::CUDA;

  options.max_solver_time_in_seconds = max_solver_time;
  options.max_num_iterations = max_num_iterations;

  ceres::Solve(options, &problem, &summary);
  if(is_kf_)
  {
    std::cout << summary.BriefReport() << std::endl;
    printf("Iterations : %d\n", static_cast<int>(summary.iterations.size()));
  }
#endif

  if(marg_flag_ == MargFlag::MARG_FIRST_NEW && summary.termination_type == ceres::NO_CONVERGENCE)
  {
    // printf("N-KF: not convergence\n");
    // opt_feature_ids_.clear();
    // return;
  }

  double2vector();
  // printf("frame_count_: %d \n", frame_count_);

  if(frame_count_ < WINDOW_SIZE)
    return;
}

void Estimator::margInThread()
{
  marginalization();

  std::vector<size_t> outliers_idxs;
  frame_manager_->outliersRejection(outliers_idxs);
  frame_manager_->removeOutlier(outliers_idxs);

  slideWindow();

  std::vector<size_t> failure_idxs;
  frame_manager_->removeFailures(failure_idxs);

  removed_feature_.clear();
  removed_feature_.insert(removed_feature_.end(), 
      outliers_idxs.begin(), outliers_idxs.end());
  removed_feature_.insert(removed_feature_.end(), 
      failure_idxs.begin(), failure_idxs.end());

  marg_done_ = true;
}

void Estimator::marginalization()
{
  if(frame_count_ < WINDOW_SIZE)
    return;

  ceres::LossFunction *loss_function;
  loss_function = new ceres::HuberLoss(1.0); // ORG
  // loss_function = new ceres::CauchyLoss(1.0);
  // loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
  // ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);

  if (marg_flag_ == MargFlag::MARG_OLD)
  {
    MarginalizationInfo *marginalization_info = new MarginalizationInfo();
    vector2double(false);

    if (last_marginalization_info_ && last_marginalization_info_->valid)
    {
      std::vector<int> drop_set;
      for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks_.size()); ++i)
      {
        if (last_marginalization_parameter_blocks_[i] == para_Pose_[0] ||
          last_marginalization_parameter_blocks_[i] == para_SpeedBias_[0])
          drop_set.push_back(i);
      }
      // construct new marginlization_factor
      MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info_);
      ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                       last_marginalization_parameter_blocks_,
                                       drop_set);
      marginalization_info->addResidualBlockInfo(residual_block_info);
    }

    if(use_imu_ && states_[1].pre_integration != nullptr 
      && states_[1].pre_integration->sum_dt_ != 0.0f)
    {
      if (states_[1].pre_integration->sum_dt_ < 10.0f)
      {
        IMUFactor* imu_factor = new IMUFactor(states_[1].pre_integration);
        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
          std::vector<double *>{para_Pose_[0], para_SpeedBias_[0], 
                  para_Pose_[1], para_SpeedBias_[1]},
          std::vector<int>{0, 1});
        marginalization_info->addResidualBlockInfo(residual_block_info);
      }
    }

    if(use_odom_ && states_[1].odom_integration != nullptr
      && states_[1].odom_integration->sum_dt_ != 0.0f)
    {
      if(states_[1].odom_integration->sum_dt_ < 10.0f)
      {  
        OdomFactor *odom_factor = new OdomFactor(states_[1].odom_integration);
        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(odom_factor, NULL,
          std::vector<double *>{para_Pose_[0], para_Pose_[1], para_OdomEx_Pose_}, 
          std::vector<int>{0});
        marginalization_info->addResidualBlockInfo(residual_block_info);
      }
    }

    addVisualResidualForMarginalize(marginalization_info, loss_function);
  
    marginalization_info->preMarginalize();
    
    marginalization_info->marginalize();

    std::unordered_map<long, double *> addr_shift;
    for (int i = 1; i <= WINDOW_SIZE; ++i)
    {
      addr_shift[reinterpret_cast<long>(para_Pose_[i])] = para_Pose_[i - 1];
      if(use_imu_)
        addr_shift[reinterpret_cast<long>(para_SpeedBias_[i])] = para_SpeedBias_[i - 1];
    }

    for (int i = 0; i < cam_size_; ++i)
      addr_shift[reinterpret_cast<long>(para_CamEx_Pose_[i])] = para_CamEx_Pose_[i];

    addr_shift[reinterpret_cast<long>(para_Td_imu_)] = para_Td_imu_;

    if(use_odom_)
    {
      addr_shift[reinterpret_cast<long>(para_OdomEx_Pose_)] = para_OdomEx_Pose_;
    }

    std::vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

    if (last_marginalization_info_)
      delete last_marginalization_info_;
    last_marginalization_info_ = marginalization_info;
    last_marginalization_parameter_blocks_ = parameter_blocks;
  }
  else if (marg_flag_ == MargFlag::MARG_FIRST_NEW)
  {
    // DO NOTHING
  }
}

void Estimator::slideWindow()
{
  if (marg_flag_ == MargFlag::MARG_OLD)
  {
    double t_0 = getTimestamp(states_[0].frame_bundle);
    if (frame_count_ == WINDOW_SIZE)
    {
      slideWindowOld();
      setMargedKF(states_[0].frame_bundle);

      std::lock_guard<std::mutex> lock(state_mutex_);

      for (int i = 0; i < WINDOW_SIZE; ++i)
      {
        states_[i].R.swap(states_[i + 1].R);
        states_[i].P.swap(states_[i + 1].P);
        states_[i].frame_bundle.swap(states_[i + 1].frame_bundle);

        if(use_imu_)
        {
          states_[i].td_imu = states_[i + 1].td_imu;

          states_[i].V.swap(states_[i + 1].V);
          states_[i].Ba.swap(states_[i + 1].Ba);
          states_[i].Bg.swap(states_[i + 1].Bg);

          states_[i].imu_dt_buf.swap(states_[i + 1].imu_dt_buf);
          states_[i].imu_acc_buf.swap(states_[i + 1].imu_acc_buf);
          states_[i].imu_gyr_buf.swap(states_[i + 1].imu_gyr_buf);

          std::swap(states_[i].pre_integration, states_[i + 1].pre_integration);
        }

        if(use_odom_)
        {
          states_[i].td_odom = states_[i + 1].td_odom;

          states_[i].odom_dt_buf.swap(states_[i + 1].odom_dt_buf);
          states_[i].odom_vel_buf.swap(states_[i + 1].odom_vel_buf);
          states_[i].odom_gyr_buf.swap(states_[i + 1].odom_gyr_buf);

          std::swap(states_[i].odom_integration, states_[i + 1].odom_integration);
        }
      }

      states_[WINDOW_SIZE].frame_bundle = states_[WINDOW_SIZE - 1].frame_bundle;
      states_[WINDOW_SIZE].P = states_[WINDOW_SIZE - 1].P;
      states_[WINDOW_SIZE].R = states_[WINDOW_SIZE - 1].R;

      if(use_imu_)
      {
        states_[WINDOW_SIZE].td_imu = td_imu_;
        
        states_[WINDOW_SIZE].V = states_[WINDOW_SIZE - 1].V;
        states_[WINDOW_SIZE].Ba = states_[WINDOW_SIZE - 1].Ba;
        states_[WINDOW_SIZE].Bg = states_[WINDOW_SIZE - 1].Bg;

        states_[WINDOW_SIZE].imu_dt_buf.clear();
        states_[WINDOW_SIZE].imu_acc_buf.clear();
        states_[WINDOW_SIZE].imu_gyr_buf.clear();

        states_[WINDOW_SIZE].pre_integration = nullptr;
      }

      if(use_odom_)
      {
        states_[WINDOW_SIZE].td_odom = td_odom_;

        states_[WINDOW_SIZE].odom_dt_buf.clear();
        states_[WINDOW_SIZE].odom_vel_buf.clear();
        states_[WINDOW_SIZE].odom_gyr_buf.clear();
        states_[WINDOW_SIZE].odom_pos_buf.clear();
        states_[WINDOW_SIZE].odom_rot_buf.clear();

        states_[WINDOW_SIZE].odom_integration = nullptr;
      }

      if(!all_image_frame_.empty())
      {
        std::map<double, ImageFrame>::iterator it_0;
        it_0 = all_image_frame_.find(t_0);
        if(it_0 != all_image_frame_.end())
          all_image_frame_.erase(all_image_frame_.begin(), it_0);
      }

      // resetKeyFrameState(states_[WINDOW_SIZE]);
    }
  }
  else if (marg_flag_ == MargFlag::MARG_FIRST_NEW)
  {
    if (frame_count_ == WINDOW_SIZE)
    {
      slideWindowFirstNew();
      setMargedKF(states_[frame_count_].frame_bundle);

      states_[frame_count_].tracked_feat_cnt = 0;
      // updateKeyFrameState(states_[frame_count_]);
    }
  }
}

void Estimator::resetKeyFrameState(const StateGroup &state)
{
  tmp_state_.reset();

  tmp_state_.frame_bundle = state.frame_bundle;

  tmp_state_.R = state.R;
  tmp_state_.P = state.P;

  if(use_imu_)
  {
    tmp_state_.td_imu = td_imu_;

    tmp_state_.V = state.V;
    tmp_state_.Ba = state.Ba;
    tmp_state_.Bg = state.Bg;

    tmp_state_.pre_integration = std::make_shared<IntegrationBase>(
          acc_prv_, gyr_prv_, state.Ba, state.Bg);
  }
  
  if(use_odom_)
  {
    tmp_state_.td_odom = td_odom_;

    tmp_state_.odom_integration = std::make_shared<IntegrationOdom>(
          odom_data_prv_.orientation, odom_data_prv_.position, odom_data_prv_.linear_velocity, 
          odom_data_prv_.angular_velocity, odom_vel_scale_, odom_gyr_scale_, td_odom_, ESTIMATE_ODOM_S3D);
  }
}

void Estimator::updateKeyFrameState(const StateGroup &state)
{
  tmp_state_.R = state.R;
  tmp_state_.P = state.P;

  if(use_imu_)
  {
    for (size_t i = 0; i < state.imu_dt_buf.size(); ++i)
    {
      const double &tmp_dt = state.imu_dt_buf[i];
      const Eigen::Vector3d &tmp_angular_velocity = state.imu_gyr_buf[i];
      const Eigen::Vector3d &tmp_linear_acceleration = state.imu_acc_buf[i];

      tmp_state_.pre_integration->pushBack(tmp_dt, 
          tmp_linear_acceleration, tmp_angular_velocity);

      tmp_state_.imu_dt_buf.push_back(tmp_dt);
      tmp_state_.imu_gyr_buf.push_back(tmp_angular_velocity);
      tmp_state_.imu_acc_buf.push_back(tmp_linear_acceleration);
    }

    tmp_state_.td_imu = td_imu_;

    tmp_state_.V = state.V;
    tmp_state_.Ba = state.Ba;
    tmp_state_.Bg = state.Bg;
  }

  if(use_odom_)
  {
    for(size_t i = 0; i < state.odom_dt_buf.size(); ++i)
    {
      const double &tmp_dt = state.odom_dt_buf[i];
      const Eigen::Vector3d &tmp_odom_vel = state.odom_vel_buf[i];
      const Eigen::Vector3d &tmp_odom_gyr = state.odom_gyr_buf[i];
      const Eigen::Vector3d &tmp_odom_pos = state.odom_pos_buf[i];;
      const Eigen::Quaterniond &tmp_odom_rot = state.odom_rot_buf[i];;

      tmp_state_.odom_integration->pushBack(tmp_dt, 
          tmp_odom_rot, tmp_odom_pos, tmp_odom_vel, tmp_odom_gyr);

      tmp_state_.odom_dt_buf.push_back(tmp_dt);
      tmp_state_.odom_vel_buf.push_back(tmp_odom_vel);
      tmp_state_.odom_gyr_buf.push_back(tmp_odom_gyr);
      tmp_state_.odom_pos_buf.push_back(tmp_odom_pos);
      tmp_state_.odom_rot_buf.push_back(tmp_odom_rot);
    }

    tmp_state_.td_odom = td_odom_;
  }
}

void Estimator::slideWindowFirstNew()
{
  frame_manager_->removeFirstNew(frame_count_);
}

void Estimator::slideWindowSecondNew()
{
  frame_manager_->removeSecondNew(frame_count_);
}

void Estimator::slideWindowOld()
{
  bool shift_depth = solver_flag_ == SolverFlag::NON_LINEAR ? true : false;
  if (shift_depth)
  {
    frame_manager_->removeBackShiftDepth();
  }
  else
  {
    frame_manager_->removeOld();
  }
}

void Estimator::addVisualResidualForOptimize(ceres::Problem &problem, ceres::LossFunction *loss_function)
{
  std::lock_guard<std::mutex> lock(frame_manager_->ftr_mutex_);

  if(options_.opt_full)
    addVisualResidualForOptimizeFull(problem, loss_function);
  else
    addVisualResidualForOptimizePart(problem, loss_function);
}

void Estimator::addVisualResidualForOptimizeFull(ceres::Problem &problem, ceres::LossFunction *loss_function)
{
  int f_m_cnt = 0;
  for(auto &feature_id : opt_feature_ids_)
  {
    const size_t &feature_index = para_feature_map_[feature_id];
    auto &obs_frames = frame_manager_->feature_per_ids_[feature_id].obs_frames;
    auto &feature_per_frame = frame_manager_->feature_per_ids_[feature_id].feature_per_frame;
     
    if(obs_frames.empty()) 
      continue;    

    int imu_i = obs_frames.front();
    double td_i = states_[imu_i].td_imu;
    auto &feature_per_frame_i = feature_per_frame[imu_i];
    const std::vector<int> obs_cam_ids_i = feature_per_frame_i.getObsCamIds();

    for(auto &cam_idx_i : obs_cam_ids_i)
    {
      const int pyr_idx_i = feature_per_frame_i.getObsPyrLevel(cam_idx_i);
      const Eigen::Vector3d &pts_i = feature_per_frame_i.getObservation(cam_idx_i);
      const Eigen::Vector3d &velocity_i = feature_per_frame_i.getObsVelocity(cam_idx_i);

      for(auto &imu_j : obs_frames)
      {
        double td_j = states_[imu_j].td_imu;
        auto &feature_per_frame_j = feature_per_frame[imu_j];
        const std::vector<int> obs_cam_ids_j = feature_per_frame_j.getObsCamIds();

        for(auto &cam_idx_j : obs_cam_ids_j)
        {
          const int pyr_idx_j = feature_per_frame_j.getObsPyrLevel(cam_idx_j);
          const Eigen::Vector3d &pts_j = feature_per_frame_j.getObservation(cam_idx_j);
          const Eigen::Vector3d &velocity_j = feature_per_frame_j.getObsVelocity(cam_idx_j);

          if(imu_i == imu_j && cam_idx_i != cam_idx_j)
          {
            SameBundleDiffCamFactor *f = new SameBundleDiffCamFactor(
                    pts_i, pts_j, velocity_i, velocity_j, td_i, td_j,
                    pyr_idx_i, pyr_idx_j);
            problem.AddResidualBlock(f, loss_function, para_CamEx_Pose_[cam_idx_i], 
                    para_CamEx_Pose_[cam_idx_j], para_InvDepth_[feature_index], para_Td_imu_);
          }
          else if(imu_i != imu_j && cam_idx_i == cam_idx_j)
          {
            DiffBundleSameCamFactor *f = new DiffBundleSameCamFactor(
                    pts_i, pts_j, velocity_i, velocity_j, td_i, td_j,
                    pyr_idx_i, pyr_idx_j);
            problem.AddResidualBlock(f, loss_function, para_Pose_[imu_i], para_Pose_[imu_j], 
                    para_CamEx_Pose_[cam_idx_i], para_InvDepth_[feature_index], para_Td_imu_);
          }
          else if(imu_i != imu_j && cam_idx_i != cam_idx_j)
          {
            DiffBundleDiffCamFactor *f = new DiffBundleDiffCamFactor(
                    pts_i, pts_j, velocity_i, velocity_j, td_i, td_j,
                    pyr_idx_i, pyr_idx_j);
            problem.AddResidualBlock(f, loss_function, para_Pose_[imu_i], para_Pose_[imu_j], 
                    para_CamEx_Pose_[cam_idx_i], para_CamEx_Pose_[cam_idx_j], 
                    para_InvDepth_[feature_index], para_Td_imu_);
          }

          if(frame_manager_->feature_per_ids_[feature_id].solve_flag == -1)
            problem.SetParameterBlockConstant(para_InvDepth_[feature_index]);

          f_m_cnt++;
        }
      }
      break;          
    }
  }

  std::cout << "addVisualResidualForOptimizeFull: "
        << "visual measurement count: " << f_m_cnt << std::endl;
}

void Estimator::addVisualResidualForOptimizePart(ceres::Problem &problem, ceres::LossFunction *loss_function)
{
  for(auto &feature_id : opt_feature_ids_)
  {
    if(frame_manager_->feature_per_ids_.count(feature_id) == 0)
      continue;

    const size_t &feature_index = para_feature_map_[feature_id];
    auto &feature_per_id = frame_manager_->feature_per_ids_[feature_id];
    auto &obs_frames = feature_per_id.obs_frames;
    auto &feature_per_frame = feature_per_id.feature_per_frame;

    if(obs_frames.empty()) 
      continue;    

    int imu_i = obs_frames.front();
    auto &feature_per_frame_i = feature_per_frame[imu_i];
    const std::vector<int> obs_cam_ids_i = feature_per_frame_i.getObsCamIds();

    for(auto &cam_idx_i : obs_cam_ids_i)
    {
      const int pyr_idx_i = feature_per_frame_i.getObsPyrLevel(cam_idx_i);
      const Eigen::Vector3d &pts_i = feature_per_frame_i.getObservation(cam_idx_i);

      for(auto &imu_j : obs_frames)
      {
        auto &feature_per_frame_j = feature_per_frame[imu_j];
        const std::vector<int> obs_cam_ids_j = feature_per_frame_j.getObsCamIds();

        for(auto &cam_idx_j : obs_cam_ids_j)
        {
          const int pyr_idx_j = feature_per_frame_j.getObsPyrLevel(cam_idx_j);
          const Eigen::Vector3d &pts_j = feature_per_frame_j.getObservation(cam_idx_j);

          if(imu_i == imu_j && cam_idx_i != cam_idx_j)
          {
            SameBundleDiffCamFactorP *f = new SameBundleDiffCamFactorP(
                    pts_i, pts_j, pyr_idx_i, pyr_idx_j);
            f->setInitExtrinsicParam(ric_[cam_idx_i], tic_[cam_idx_i], 
                                    ric_[cam_idx_j], tic_[cam_idx_j]);
            problem.AddResidualBlock(f, loss_function, para_InvDepth_[feature_index]);
          }
          else if(imu_i != imu_j && cam_idx_i == cam_idx_j)
          {
            DiffBundleSameCamFactorP *f = new DiffBundleSameCamFactorP(
                    pts_i, pts_j, pyr_idx_i, pyr_idx_j);
            f->setInitExtrinsicParam(ric_[cam_idx_i], tic_[cam_idx_i], 
                                    ric_[cam_idx_i], tic_[cam_idx_i]);
            problem.AddResidualBlock(f, loss_function, para_Pose_[imu_i], 
                    para_Pose_[imu_j], para_InvDepth_[feature_index]);
          }
          else if(imu_i != imu_j && cam_idx_i != cam_idx_j)
          {
            DiffBundleDiffCamFactorP *f = new DiffBundleDiffCamFactorP(
                    pts_i, pts_j, pyr_idx_i, pyr_idx_j);
            f->setInitExtrinsicParam(ric_[cam_idx_i], tic_[cam_idx_i], 
                                    ric_[cam_idx_j], tic_[cam_idx_j]);
            problem.AddResidualBlock(f, loss_function, para_Pose_[imu_i], 
                    para_Pose_[imu_j], para_InvDepth_[feature_index]);
          }

          if(feature_per_id.solve_flag == -1)
            problem.SetParameterBlockConstant(para_InvDepth_[feature_index]);
        }
      }
      break;          
    }
  }
}

void Estimator::addVisualResidualForMarginalize(
  MarginalizationInfo *marginalization_info, 
  ceres::LossFunction *loss_function)
{
  std::lock_guard<std::mutex> lock(frame_manager_->ftr_mutex_);

  if(options_.opt_full)
    addVisualResidualForMarginalizeFull(marginalization_info, loss_function);
  else
    addVisualResidualForMarginalizePart(marginalization_info, loss_function);
}

void Estimator::addVisualResidualForMarginalizeFull(
  MarginalizationInfo *marginalization_info, 
  ceres::LossFunction *loss_function)
{
  for(auto &feature_id : opt_feature_ids_)
  {
    const size_t &feature_index = para_feature_map_[feature_id];
    auto &obs_frames = frame_manager_->feature_per_ids_[feature_id].obs_frames;
    
    if(obs_frames.empty()) 
      continue;    

    int imu_i = obs_frames.front();
    if (imu_i != 0) continue;
    
    auto &feature_per_frame = frame_manager_->feature_per_ids_[feature_id].feature_per_frame;

    double td_i = states_[imu_i].td_imu;
    auto &feature_per_frame_i = feature_per_frame[imu_i];
    const std::vector<int> obs_cam_ids_i = feature_per_frame_i.getObsCamIds();

    for(auto &cam_idx_i : obs_cam_ids_i)
    {
      const int pyr_idx_i = feature_per_frame_i.getObsPyrLevel(cam_idx_i);
      const Eigen::Vector3d &pts_i = feature_per_frame_i.getObservation(cam_idx_i);
      const Eigen::Vector3d &velocity_i = feature_per_frame_i.getObsVelocity(cam_idx_i);
      
      for(auto &imu_j : obs_frames)
      {
        double td_j = states_[imu_j].td_imu;
        auto &feature_per_frame_j = feature_per_frame[imu_j];
        const std::vector<int> obs_cam_ids_j = feature_per_frame_j.getObsCamIds();

        for(auto &cam_idx_j : obs_cam_ids_j)
        {
          const int pyr_idx_j = feature_per_frame_j.getObsPyrLevel(cam_idx_j);
          const Eigen::Vector3d &pts_j = feature_per_frame_j.getObservation(cam_idx_j);
          const Eigen::Vector3d &velocity_j = feature_per_frame_j.getObsVelocity(cam_idx_j);

          if(imu_i == imu_j && cam_idx_i != cam_idx_j)
          {
            SameBundleDiffCamFactor *f = new SameBundleDiffCamFactor(
                      pts_i, pts_j, velocity_i, velocity_j, td_i, td_j,
                      pyr_idx_i, pyr_idx_j);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                      std::vector<double *>{para_CamEx_Pose_[cam_idx_i], para_CamEx_Pose_[cam_idx_j], 
                              para_InvDepth_[feature_index], para_Td_imu_},
                      std::vector<int>{2});
            marginalization_info->addResidualBlockInfo(residual_block_info);
          }
          else if(imu_i != imu_j && cam_idx_i == cam_idx_j)
          {
            DiffBundleSameCamFactor *f = new DiffBundleSameCamFactor(
                        pts_i, pts_j, velocity_i, velocity_j, td_i, td_j,
                        pyr_idx_i, pyr_idx_j);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                        std::vector<double *>{para_Pose_[imu_i], para_Pose_[imu_j], 
                                para_CamEx_Pose_[cam_idx_i], para_InvDepth_[feature_index], 
                                para_Td_imu_},
                        std::vector<int>{0, 3});
            marginalization_info->addResidualBlockInfo(residual_block_info); 
          }
          else if(imu_i != imu_j && cam_idx_i != cam_idx_j)
          {
            DiffBundleDiffCamFactor *f = new DiffBundleDiffCamFactor(
                    pts_i, pts_j, velocity_i, velocity_j, td_i, td_j,
                    pyr_idx_i, pyr_idx_j);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                      std::vector<double *>{para_Pose_[imu_i], para_Pose_[imu_j], para_CamEx_Pose_[cam_idx_i], 
                              para_CamEx_Pose_[cam_idx_j], para_InvDepth_[feature_index], para_Td_imu_},
                      std::vector<int>{0, 4});
            marginalization_info->addResidualBlockInfo(residual_block_info);  
          }
        }
      }
      break;
    }
  }
}

void Estimator::addVisualResidualForMarginalizePart(
  MarginalizationInfo *marginalization_info, 
  ceres::LossFunction *loss_function)
{
  for(size_t lm_idx = 0; lm_idx < opt_feature_ids_.size(); ++lm_idx)
  {
    size_t feature_id = opt_feature_ids_[lm_idx];
    if(frame_manager_->feature_per_ids_.count(feature_id) == 0)
      continue;

    auto &feature_per_id = frame_manager_->feature_per_ids_[feature_id];
    const size_t &feature_index = para_feature_map_[feature_id];
    auto &obs_frames = feature_per_id.obs_frames;
    
    if(obs_frames.empty()) 
      continue;    

    int imu_i = obs_frames.front();
    if (imu_i != 0) continue;
    
    auto &feature_per_frame = feature_per_id.feature_per_frame;
    auto &feature_per_frame_i = feature_per_frame[imu_i];
    const std::vector<int> obs_cam_ids_i = feature_per_frame_i.getObsCamIds();

    for(auto &cam_idx_i : obs_cam_ids_i)
    {
      int pyr_idx_i = feature_per_frame_i.getObsPyrLevel(cam_idx_i);
      const Eigen::Vector3d &pts_i = feature_per_frame_i.getObservation(cam_idx_i);

      for(auto &imu_j : obs_frames)
      {
        auto &feature_per_frame_j = feature_per_frame[imu_j];
        const std::vector<int> obs_cam_ids_j = feature_per_frame_j.getObsCamIds();

        for(auto &cam_idx_j : obs_cam_ids_j)
        {
          int pyr_idx_j = feature_per_frame_j.getObsPyrLevel(cam_idx_j);
          const Eigen::Vector3d &pts_j = feature_per_frame_j.getObservation(cam_idx_j);

          if(imu_i == imu_j && cam_idx_i != cam_idx_j)
          {
            SameBundleDiffCamFactorP *f = new SameBundleDiffCamFactorP(
                      pts_i, pts_j, pyr_idx_i, pyr_idx_j);
            f->setInitExtrinsicParam(ric_[cam_idx_i], tic_[cam_idx_i], 
                      ric_[cam_idx_j], tic_[cam_idx_j]);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                      std::vector<double *>{para_InvDepth_[feature_index]},
                      std::vector<int>{0});
            marginalization_info->addResidualBlockInfo(residual_block_info);
          }
          else if(imu_i != imu_j && cam_idx_i == cam_idx_j)
          {
            DiffBundleSameCamFactorP *f = new DiffBundleSameCamFactorP(
                      pts_i, pts_j, pyr_idx_i, pyr_idx_j);
            f->setInitExtrinsicParam(ric_[cam_idx_i], tic_[cam_idx_i], 
                      ric_[cam_idx_i], tic_[cam_idx_i]);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                      std::vector<double *>{para_Pose_[imu_i], para_Pose_[imu_j], 
                              para_InvDepth_[feature_index]},
                      std::vector<int>{0, 2});
            marginalization_info->addResidualBlockInfo(residual_block_info); 
          }
          else if(imu_i != imu_j && cam_idx_i != cam_idx_j)
          {
            DiffBundleDiffCamFactorP *f = new DiffBundleDiffCamFactorP(
                      pts_i, pts_j, pyr_idx_i, pyr_idx_j);
            f->setInitExtrinsicParam(ric_[cam_idx_i], tic_[cam_idx_i], 
                      ric_[cam_idx_j], tic_[cam_idx_j]);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                      std::vector<double *>{para_Pose_[imu_i], para_Pose_[imu_j], 
                              para_InvDepth_[feature_index]},
                      std::vector<int>{0, 2});
            marginalization_info->addResidualBlockInfo(residual_block_info);  
          }
        }
      }
      break;
    }
  }
}

void Estimator::addImageFrame(const FrameBundlePtr &new_frames)
{
  if(solver_flag_ == SolverFlag::NON_LINEAR)
    return;

  ImageFrame image_frame(new_frames);
  double ts = getTimestamp(new_frames);
  
  if(use_imu_ || use_odom_)
  {
    if(use_imu_)
    {
      image_frame.pre_integration = tmp_pre_integration_;
      tmp_pre_integration_ = std::make_shared<IntegrationBase>(
        acc_prv_, gyr_prv_, states_[frame_count_].Ba, states_[frame_count_].Bg);
    }

    if(use_odom_)
    {
      image_frame.odom_integration = tmp_odom_integration_;
      tmp_odom_integration_ = std::make_shared<IntegrationOdom>(
                odom_data_prv_.orientation, odom_data_prv_.position, odom_data_prv_.linear_velocity, 
                odom_data_prv_.angular_velocity, odom_vel_scale_, odom_gyr_scale_, td_odom_, ESTIMATE_ODOM_S3D);
      setOdomWeight(tmp_odom_integration_, states_[frame_count_].tracked_feat_cnt);
    }

    image_frame.setPose(states_[frame_count_].R, states_[frame_count_].P);
    all_image_frame_.insert(std::make_pair(ts, image_frame));
  }
}

bool checkLandmarkStatus(const FrameBundlePtr &new_frames)
{
  bool res = false;
  for(unsigned int cam_idx = 0; cam_idx < new_frames->size(); ++cam_idx)
  {
    const FramePtr &frame = new_frames->at(cam_idx);

    size_t feature_size = numFeatures(frame);
    for(size_t ftr_idx = 0; ftr_idx < feature_size; ++ftr_idx)
    {
      if(ftr_idx >= (size_t)frame->track_id_vec_.size() 
        || ftr_idx >= frame->landmark_vec_.size())
        break;

      if(frame->landmark_vec_[ftr_idx] != nullptr)
      {
        res = true;
        break;
      }
    }
  }

  return res;
}

void Estimator::addInitFrameBundle(const FrameBundlePtr &new_frames)
{
  scope_color(ANSI_COLOR_YELLOW);

  printf("addInitFrameBundle: frame_count: %d; timestamp: %.9lf\n", 
        frame_count_, getTimestamp(new_frames));

  is_kf_ = true;
  cur_ts_ = getTimestamp(new_frames);

  processImuAndOdom(new_frames);
  prv_ts_ = cur_ts_;

  std::lock_guard<std::mutex> lock(process_mutex_);

  if(PIPLINE_TYPE == PipType::kMono || PIPLINE_TYPE == PipType::kRgbd)
    monoInitialize(new_frames);
  else if(PIPLINE_TYPE == PipType::kStereo || PIPLINE_TYPE == PipType::kStereoWithTwoFisheye)
    stereoInitialize(new_frames);
  else if(PIPLINE_TYPE == PipType::kTripleWithStereo)
    tripleWithStereoInitialize(new_frames);
  else if(PIPLINE_TYPE == PipType::kTripleWithDepth)
    tripleWithDepthInitialize(new_frames);

  updateInitFrameBundles(new_frames);
}

void Estimator::initWindowState()
{
  if(frame_count_ < WINDOW_SIZE)
  {
    frame_count_++;
    StateGroup &prev_state = states_[frame_count_ - 1];

    states_[frame_count_].P = prev_state.P;
    states_[frame_count_].V = prev_state.V;
    states_[frame_count_].R = prev_state.R;
    states_[frame_count_].Ba = prev_state.Ba;
    states_[frame_count_].Bg = prev_state.Bg;
  }
}

void Estimator::monoInitialize(const FrameBundlePtr &new_frames)
{
  std::cout << "imu predicted pose: \n";
  printPose(new_frames);

  bool res = frame_manager_->checkParallax(frame_count_, new_frames);
  if(!res) return;

  frame_manager_->addMonoFrameBundle(frame_count_, new_frames);
  states_[frame_count_].frame_bundle = new_frames;
  marg_flag_ = MargFlag::MARG_OLD;
  addImageFrame(new_frames);

  frame_manager_->drawMonoFeatureTrack(frame_count_ - 1, frame_count_);

  if(frame_count_ == WINDOW_SIZE)
  {
    bool result = false;
    if((getTimestamp(new_frames) - initial_ts_) > 0.1)
    {
      int marg_flag = 0;
      initializer_->setImuAvailable(use_imu_);
      initializer_->setOdomAvailable(use_odom_);
      result = initializer_->initialStructure(frame_count_, 
                              all_image_frame_, marg_flag);
      initial_ts_ = getTimestamp(new_frames);   
      // marg_flag_ = marg_flag;
    }
    if(result)
    {
      optimization();
      marginalization();
      solver_flag_ = SolverFlag::NON_LINEAR;
      slideWindow(); 
      printf("Initialization finish!\n");
      cv::destroyAllWindows();
    }
    else
    {
      slideWindow();
    }
  }

  initWindowState();
}

void Estimator::stereoInitialize(const FrameBundlePtr &new_frames)
{  
  states_[frame_count_].frame_bundle = new_frames;

  frame_manager_->addFrameBundle(frame_count_, new_frames);

  frame_manager_->getTrackingState(frame_count_);

  if(states_[frame_count_].tracked_feat_cnt > options_.quality_min_fts)
  {
    Transformation T = getImu2WorldTrans(new_frames);
    states_[frame_count_].R = getRotationMatrix(T);
    states_[frame_count_].P = getPosition(T);
  }

  addImageFrame(new_frames);
  marg_flag_ = MargFlag::MARG_OLD;

  // if(!use_imu_ && !use_odom_)
  //   optimization();

  if (frame_count_ == WINDOW_SIZE)
  {
    initializer_->setImuAvailable(use_imu_);
    initializer_->setOdomAvailable(use_odom_);

    if(use_imu_)
    {
      bool align_result = false;
      // TODO
      // align_result = initializer_->visualInitialAlign(frame_count_, all_image_frame_);
      if(!align_result)
      {
        printf("Warning: visualInitialAlign failed\n");
        InitialAlignment::Ptr init_align = std::make_shared<InitialAlignment>();
        init_align->solveGyroscopeBias(all_image_frame_, states_);
        for(int i = 0; i <= WINDOW_SIZE; ++i)
        {
          if(std::isnan(states_[i].Bg.norm()))
            states_[i].Bg.setConstant(2e-3);

          if(states_[i].pre_integration != nullptr)
          {
            Eigen::Vector3d tmp_ba = Eigen::Vector3d::Zero();
            states_[i].pre_integration->repropagate(tmp_ba, states_[i].Bg);
          }
        }
      }
    }

    /*
    if(use_odom_)
    {
      InitialAlignment::Ptr init_align = std::make_shared<InitialAlignment>();
      init_align->solveGyroscopeBias(all_image_frame_, states_, rio_, tio_);
      for(int i = 0; i <= WINDOW_SIZE; ++i)
      {
        if(std::isnan(states_[i].odom_Bg.norm()))
          states_[i].odom_Bg.setConstant(2e-3);
        if(states_[i].odom_integration != nullptr)
          states_[i].odom_integration->repropagate(states_[i].odom_Bg);
      }
    }
    */
    
    std::cout << "Before optimize:\n";
    printPoses();

    // frame_manager_->triangulate();
    stateCheck();
    optimization();
    marginalization();
    reTriangulate();

    std::cout << "After optimize:\n";
    printPoses();

    std::vector<size_t> outliers_idxs;
    frame_manager_->outliersRejection(outliers_idxs);    
    frame_manager_->removeOutlier(outliers_idxs);

    solver_flag_ = SolverFlag::NON_LINEAR;
    // std::lock_guard<std::mutex> lock(init_mutex_);
    // con_.wait(lock);
    slideWindow();

    std::vector<size_t> failure_idxs;
    frame_manager_->removeFailures(failure_idxs);

    std::cout << "feature size: " << frame_manager_->feature_per_ids_.size() << "; ";
    std::cout << "remove index size: " << outliers_idxs.size() << std::endl;
    std::cout << "failure index size: " << failure_idxs.size() << std::endl;
    
    printf("Initialization finish!\n");
  }

  initWindowState();
}

// the triple camera consists of one stereo camera and one mono camera
void Estimator::tripleWithStereoInitialize(const FrameBundlePtr &new_frames)
{
  Transformation T = getImu2WorldTrans(new_frames);
  states_[frame_count_].R = getRotationMatrix(T);
  states_[frame_count_].P = getPosition(T);
  states_[frame_count_].frame_bundle = new_frames;

  /*
  // WCP TODO: something should to change
  // one stereo camera(front) and one mono camera(behind)
  BundleId bundle_id = new_frames->getBundleId();
  std::vector<FramePtr> mono_frames{new_frames->at(2)}; 
  FrameBundlePtr mono_frame_bundle(new FrameBundle(mono_frames, bundle_id));
  trackers_.front()->trackAndDetect(mono_frame_bundle);
  */
  frame_manager_->addFrameBundle(frame_count_, new_frames);
  marg_flag_ = MargFlag::MARG_OLD;
  
  addImageFrame(new_frames);

  // optimization(); // BAD should check

  if (frame_count_ == WINDOW_SIZE)
  {
    if(use_imu_)
    {
      InitialAlignment::Ptr init_align = std::make_shared<InitialAlignment>();
      init_align->solveGyroscopeBias(all_image_frame_, states_);
      for(int i = 0; i <= WINDOW_SIZE; ++i)
      {
        if(std::isnan(states_[i].Bg.norm()))
          states_[i].Bg.setConstant(2e-3);
        if(states_[i].pre_integration != nullptr)
          states_[i].pre_integration->repropagate(Eigen::Vector3d::Zero(), states_[i].Bg);
      }
    }

    frame_manager_->triangulate();
    optimization();
    marginalization();
    solver_flag_ = SolverFlag::NON_LINEAR;
    
    slideWindow();
    printf("Initialization finish!\n");
  }

  initWindowState();
}

void Estimator::tripleWithDepthInitializeWithSfm(const FrameBundlePtr &new_frames)
{
  bool res = frame_manager_->checkParallax(frame_count_, new_frames);
  if(!res) return;

  int cam_idx = 0;
  int matches_num = 0;
  int matches_num_thresh = 25;

  states_[frame_count_].frame_bundle = new_frames;
  frame_manager_->addMonoFrameBundle(frame_count_, new_frames);    
  bool pnp_res = frame_manager_->getPoseWithPnPRansac(frame_count_, cam_idx, 
                    matches_num_thresh, matches_num, 
                    states_[frame_count_].R, states_[frame_count_].P); 
  std::cout << "matches_num: " << matches_num << std::endl;
  if(!pnp_res && matches_num != 0)
  {
    reset();
    return;
  }

  // bool marg_old = frame_manager_->checkParallax(frame_count_, cam_idx);
  frame_manager_->drawMonoFeatureTrack(frame_count_ - 1, frame_count_);
  marg_flag_ = MargFlag::MARG_OLD;
  printPose(frame_count_);

  addImageFrame(new_frames);

  if (frame_count_ == WINDOW_SIZE)
  {
    // bool result = false;
    // if(getTimestamp(new_frames) - initial_ts_ > 0.1)
    // {
    //     result = initialStructure();
    //     initial_ts_ = getTimestamp(new_frames);   
    // }
    // if(result)
    {
      if(use_imu_)
      {
        InitialAlignment::Ptr init_align = std::make_shared<InitialAlignment>();
        init_align->solveGyroscopeBias(all_image_frame_, states_);
        for(int i = 0; i <= WINDOW_SIZE; ++i)
        {
          if(std::isnan(states_[i].Bg.norm()))
            states_[i].Bg.setConstant(2e-3);
          if(states_[i].pre_integration != nullptr)
            states_[i].pre_integration->repropagate(Eigen::Vector3d::Zero(), states_[i].Bg);
        }
      }
      frame_manager_->triangulate();

      std::cout << "Before optimize:\n";
      printPoses();

      optimization();
      marginalization();

      std::vector<size_t> outliers_idxs;
      frame_manager_->outliersRejection(outliers_idxs);    
      frame_manager_->removeOutlier(outliers_idxs);

      solver_flag_ = SolverFlag::NON_LINEAR;
      
      slideWindow();

      std::vector<size_t> failure_idxs;
      frame_manager_->removeFailures(failure_idxs);

      // std::cout << "After optimize:\n";
      // printPoses();
    }
    // else
    //     slideWindow();
  }

  initWindowState();
}


// the triple camera consists of one rgbd camera(front) and two fisheye camera(side)
void Estimator::tripleWithDepthInitialize(const FrameBundlePtr &new_frames)
{
  bool has_landmark = checkLandmarkStatus(new_frames);
  if(!has_landmark)
  {
    tripleWithDepthInitializeWithSfm(new_frames);
    return ;
  }

  Transformation T = getImu2WorldTrans(new_frames);
  states_[frame_count_].R = getRotationMatrix(T);
  states_[frame_count_].P = getPosition(T);
  states_[frame_count_].frame_bundle = new_frames;

  frame_manager_->addFrameBundle(frame_count_, new_frames);

  marg_flag_ = MargFlag::MARG_OLD;
  printPose(frame_count_);

  addImageFrame(new_frames);

  if (frame_count_ == WINDOW_SIZE)
  {
    if(use_imu_)
    {
      InitialAlignment::Ptr init_align = std::make_shared<InitialAlignment>();
      init_align->solveGyroscopeBias(all_image_frame_, states_);
      for(int i = 0; i <= WINDOW_SIZE; ++i)
      {
        if(std::isnan(states_[i].Bg.norm()))
          states_[i].Bg.setConstant(2e-3);
        if(states_[i].pre_integration != nullptr)
          states_[i].pre_integration->repropagate(Eigen::Vector3d::Zero(), states_[i].Bg);
      }
    }

    std::cout << "Before optimize:\n";
    printPoses();
    // frame_manager_->triangulate();

    optimization();
    marginalization();

    std::vector<size_t> outliers_idxs;
    frame_manager_->outliersRejection(outliers_idxs);    
    frame_manager_->removeOutlier(outliers_idxs);

    solver_flag_ = SolverFlag::NON_LINEAR;
    
    slideWindow();

    std::vector<size_t> failure_idxs;
    frame_manager_->removeFailures(failure_idxs);

    std::cout << "After optimize:\n";
    printPoses();
  }

  initWindowState();
}


void Estimator::addFrameBundle(const FrameBundlePtr& new_frames)
{      
  if(!BACKEND_OPT)
    return;

  {
    std::lock_guard<std::mutex> lock(buf_mutex_);
    frame_bundles_buf_.push(new_frames);
  }

  if(!MULTIPLE_THREAD)
  {
    start_thread_ = true;
    process();
  }
}

void Estimator::process()
{
  while(start_thread_)
  {
    if(!start_thread_)
      break;

    if(frame_bundles_buf_.empty())
    {
      if (!MULTIPLE_THREAD)
        break;
      else
      {
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
        continue;
      }
    }

    FrameBundlePtr frame_bundle;

    {
      std::lock_guard<std::mutex> lock(buf_mutex_);
      while(!frame_bundles_buf_.empty())
      {
        frame_bundle = frame_bundles_buf_.front();
        frame_bundles_buf_.pop();
      }
    }
    
    is_kf_ = isKeyframe(frame_bundle);
    cur_ts_ = getTimestamp(frame_bundle);

    if(!marg_done_ && !is_kf_)
    {
      if (!MULTIPLE_THREAD)
        break;
      else
        continue;
    }

    while(!marg_done_)
    {
      std::cout << "Wait for marg done\n";
      std::chrono::milliseconds dura(5);
      std::this_thread::sleep_for(dura);
    }      

    if(is_kf_)
      printf("............................................. dt: %.9f\n", cur_ts_ - prv_ts_);

    // when initiliazed is done, this would happend
    if(cur_ts_ <= prv_ts_)
    {
      if (!MULTIPLE_THREAD)
        break;
      else
        continue;
    }

    processImuAndOdom(frame_bundle);
    prv_ts_ = cur_ts_;
    
    if(options_.opt_kf_only && !is_kf_)
    {
      if(!MULTIPLE_THREAD)
        break;
      else
        continue;
    }

    processFrameBundle(frame_bundle);

    scope_color(ANSI_COLOR_BLUE);     
  }
}

void Estimator::processFrameBundle(const FrameBundlePtr& new_frames)
{
  states_[frame_count_].frame_bundle = new_frames;

  frame_manager_->addFrameBundle(frame_count_, new_frames);
  frame_manager_->getTrackingState(frame_count_);
  // frame_manager_->getTrackingState(frame_bundle, new_frames);

  Transformation T_w_b = getImu2WorldTrans(new_frames);
  Eigen::Matrix3d rot_tmp = getRotationMatrix(T_w_b);
  Eigen::Vector3d trans_tmp = getPosition(T_w_b);

  if(states_[frame_count_].tracked_feat_cnt > options_.quality_min_fts)
  {
    states_[frame_count_].R = getRotationMatrix(T_w_b);
    states_[frame_count_].P = getPosition(T_w_b);
  }

  if(is_kf_)
    std::cout << "Before optimize: " << states_[frame_count_].P.transpose() << "\n";

  addImageFrame(new_frames);

  marg_flag_ = is_kf_ ? MargFlag::MARG_OLD : MargFlag::MARG_FIRST_NEW;

  stateCheck();

  initPoseByPnP();

  optimization();
  
  if(is_kf_)
    std::cout << "After optimize: " << states_[WINDOW_SIZE].P.transpose() << "\n";

  if (failureDetection())
  {
    scope_color(ANSI_COLOR_RED_BOLD);
    printf("Backend failure detection!\n");
    failure_occur_ = true;
    // reset();
    // return;
  }

  if(options_.marg_in_thread)
  {
    reTriangulate();
    updateFrameBundles();

    marg_done_ = false;
    marg_thread_ = std::thread(&Estimator::margInThread, this);
    marg_thread_.detach();
  }
  else
  {
    marginalization();

    reTriangulate();

    std::vector<size_t> outliers_idxs;
    frame_manager_->outliersRejection(outliers_idxs);    
    frame_manager_->removeOutlier(outliers_idxs);

    slideWindow();
    
    std::vector<size_t> failure_idxs;
    frame_manager_->removeFailures(failure_idxs);

    removed_feature_.clear();
    removed_feature_.insert(removed_feature_.end(), 
        outliers_idxs.begin(), outliers_idxs.end());
    removed_feature_.insert(removed_feature_.end(), 
        failure_idxs.begin(), failure_idxs.end());

    // std::cout << "feature size: " << frame_manager_->feature_per_ids_.size() << "; ";
    // std::cout << "remove index size: " << outliers_idxs.size() << "\n";

    updateFrameBundles();
  }

  printExtrinsic();

  if(MULTIPLE_THREAD)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    frame_ts_ = cur_ts_;
    rot_org_ = rot_tmp;
    trans_org_ = trans_tmp;

    rot_opt_ = states_[WINDOW_SIZE].R;
    trans_opt_ = states_[WINDOW_SIZE].P;
    
    feature_per_ids_ = frame_manager_->feature_per_ids_;
  }
}

void Estimator::stateCheck()
{
  return;

  if(!is_kf_)
   return;

  std::vector<int> obs_count(WINDOW_SIZE + 1, 0);
  for(auto &kv : frame_manager_->feature_per_ids_)
  {
    auto &feature_per_id = kv.second;
    auto &obs_frames = feature_per_id.obs_frames;
    obs_count[obs_frames.size() - 1]++;
  }

  std::cout << "[obs size, count]: ";
  for(int i = 0; i <= WINDOW_SIZE; ++i)
    std::cout << "[" << i + 1 << ": " << obs_count[i] << "] ";
  std::cout << std::endl;
}

void Estimator::updateInitFrameBundles(const FrameBundlePtr &new_frames)
{    
  if(solver_flag_ != SolverFlag::NON_LINEAR)
    return;

  last_R_ = states_[WINDOW_SIZE].R;
  last_P_ = states_[WINDOW_SIZE].P;
  last_R0_ = states_[0].R;
  last_P0_ = states_[0].P;

  bool has_landmark = checkLandmarkStatus(new_frames);

  if(PIPLINE_TYPE == PipType::kMono || PIPLINE_TYPE == PipType::kRgbd)
  {
    updateMonoInitFrameBundles();
  }
  else if(PIPLINE_TYPE == PipType::kStereo 
       || PIPLINE_TYPE == PipType::kTripleWithStereo)
  {
    updateFrameBundles();
  }
  else if(PIPLINE_TYPE == PipType::kTripleWithDepth)
  {
    if(has_landmark)
    {
      updateFrameBundles();
    }
    else
    {
      updateMonoInitFrameBundles();
    }
  }
}

void Estimator::updateFramePose()
{
  for(int i = 0; i <= WINDOW_SIZE; ++i)
  {
    if(states_[i].frame_bundle == nullptr)
      continue;

    FrameBundlePtr &frame_bundle = states_[i].frame_bundle;
    Eigen::Vector3d P_w_i = states_[i].P;
    Eigen::Quaterniond Q_w_i(states_[i].R);
    Q_w_i.normalize();

    Transformation T_w_i(P_w_i, Q_w_i);
    setImu2WorldTrans(frame_bundle, T_w_i);
  }
}

void Estimator::updateLandmarks()
{
  std::lock_guard<std::mutex> lock(frame_manager_->ftr_mutex_);
  
  for(size_t lm_idx = 0; lm_idx < frame_manager_->feature_ids_.size(); ++lm_idx)
  {
    size_t feature_id = frame_manager_->feature_ids_[lm_idx];
    if(frame_manager_->feature_per_ids_.count(feature_id) == 0)
      continue;

    auto &feature_per_id = frame_manager_->feature_per_ids_[feature_id];
    if(marg_flag_ == MargFlag::MARG_FIRST_NEW)
    {
      auto iter = std::find(opt_feature_ids_.begin(), opt_feature_ids_.end(), feature_id);
      if(iter == opt_feature_ids_.end())
        continue;
    }

    const int &solve_flag = feature_per_id.solve_flag;
    const double &estimated_depth = feature_per_id.estimated_depth;

    if(std::abs(solve_flag) != 1 || estimated_depth < 0.0f || estimated_depth == INIT_DEPTH)
      continue;

    auto &obs_frames = feature_per_id.obs_frames;
    auto &feature_per_frame = feature_per_id.feature_per_frame;

    if(obs_frames.empty()) 
      continue;    

    int frame_id = obs_frames.front();
    auto &feature_per_cam = feature_per_frame[frame_id];
    const std::vector<int> obs_cam_ids = feature_per_cam.getObsCamIds();

    const FrameBundlePtr &frame_bundle = states_[frame_id].frame_bundle;

    int cam_idx = obs_cam_ids.front();
    const FramePtr &frame = frame_bundle->at(cam_idx);
    const Eigen::Vector3d &pts = feature_per_cam.getObservation(cam_idx);

    Eigen::Vector3d pts_cam = pts * estimated_depth;
    Eigen::Vector3d pts_world = frame->T_world_cam() * pts_cam;
    
    for(auto &frame_id : obs_frames)
    {
      auto &feature_per_cam = feature_per_frame[frame_id];
      const std::vector<int> obs_cam_ids = feature_per_cam.getObsCamIds();
      const FrameBundlePtr &frame_bundle = states_[frame_id].frame_bundle;

      for(auto &cam_idx : obs_cam_ids)
      {
        size_t ftr_idx = feature_per_cam.getObsFtrIdx(cam_idx);
        const FramePtr &frame = frame_bundle->at(cam_idx);
        if(frame->landmark_vec_[ftr_idx] != nullptr)
        {
          setLandmarkPos(frame->landmark_vec_[ftr_idx], pts_world);
        }
        else
        {
          // WCP TODO: 
          /*
          PointPtr point = std::make_shared<Point>(feature_id, pts_world);
          frame->track_id_vec_(ftr_idx) = feature_id;
          frame->landmark_vec_[ftr_idx] = point;

          point->addObservation(frame, ftr_idx);
          if(svo::isCorner(frame->type_vec_[ftr_idx]))
              frame->type_vec_[ftr_idx] = FeatureType::kCorner;
          else if(svo::isEdgelet(frame->type_vec_[ftr_idx]))
              frame->type_vec_[ftr_idx] = FeatureType::kEdgelet;

          for(unsigned int j = 1; j < obs_cam_ids.size(); ++j)
          {
            int cam_idx_j = obs_cam_ids[j];
            const FramePtr &frame_j = frame_bundle->at(cam_idx_j);
            frame_j->track_id_vec_(ftr_idx_j) = feature_id;
            frame_j->landmark_vec_[ftr_idx_j] = point;

            point->addObservation(frame_j, ftr_idx_j);
            if(svo::isCorner(frame_j->type_vec_[ftr_idx_j]))
                frame_j->type_vec_[ftr_idx] = FeatureType::kCorner;
            else if(svo::isEdgelet(frame_j->type_vec_[ftr_idx_j]))
                frame_j->type_vec_[ftr_idx_j] = FeatureType::kEdgelet;
          }
          break;
          */
        }
      }
    }
  }
}

void Estimator::updateFrameBundles()
{
  if(solver_flag_ == SolverFlag::INITIAL)
    return;
  // static double last_update_ts = 0.0;

  // if(last_update_ts == getTimestamp(states_[WINDOW_SIZE].frame_bundle))
  //   return;
  // last_update_ts = getTimestamp(states_[WINDOW_SIZE].frame_bundle);

  last_R_ = states_[WINDOW_SIZE].R;
  last_P_ = states_[WINDOW_SIZE].P;
  last_R0_ = states_[0].R;
  last_P0_ = states_[0].P;

  // update keyframe status
  updateFramePose();
  // update landmark
  updateLandmarks();

  if(0 && MULTIPLE_THREAD)
  {
    // no use, can be commented out
    optimizeStructure(states_[WINDOW_SIZE - 1].frame_bundle, -1, 5);
  }

  // delete outlier
  bool delete_outlier = false;
  if(delete_outlier)
  {
    for(auto feature_id : removed_feature_)
    {
      auto &obs_frames = frame_manager_->feature_per_ids_[feature_id].obs_frames;
      auto &feature_per_frame = frame_manager_->feature_per_ids_[feature_id].feature_per_frame;

      for(auto imu_i : obs_frames)
      {
        FrameBundlePtr &frame_bundle = states_[imu_i].frame_bundle;
        const std::vector<int> obs_cam_ids = feature_per_frame[imu_i].getObsCamIds();
        for(auto &cam_idx : obs_cam_ids)
        {
          size_t ftr_idx = feature_per_frame[imu_i].getObsFtrIdx(cam_idx);
          const FramePtr &frame = frame_bundle->at(cam_idx);

          frame->type_vec_[ftr_idx] = FeatureType::kOutlier;
          frame->seed_ref_vec_[ftr_idx].keyframe.reset();
          removeLandmarkObs(frame->landmark_vec_[ftr_idx], frame);
          frame->landmark_vec_[ftr_idx] = nullptr;
        }
      }
    }
  }

  removed_feature_.clear();

  // WCP TODO
  // for(int i = 0; i <= WINDOW_SIZE; ++i)
  // {
  //   FrameBundlePtr &frame_bundle = states_[i].frame_bundle;
  //   for(size_t j = 0; j < frame_bundle->size(); ++j)
  //   {
  //     const FramePtr &frame = frame_bundle->at(j);
  //     resetKeyPoints(frame);
  //     setKeyPoints(frame);
  //   }
  // }
}


void Estimator::updateMonoInitFrameBundles()
{
  if(solver_flag_ == SolverFlag::INITIAL)
    return;
  
  updateFramePose();

  int point_cnt = 0;
  std::vector<int> frame_cnt(WINDOW_SIZE + 1, 0);
  for(auto &kv : frame_manager_->feature_per_ids_)
  {
    size_t feature_id = kv.first;
    auto &feature_per_id = kv.second;
    int solve_flag = feature_per_id.solve_flag;
    auto &obs_frames = feature_per_id.obs_frames;
    auto &feature_per_frame = feature_per_id.feature_per_frame;
    double estimated_depth = feature_per_id.estimated_depth;

    if(solve_flag != 1 || estimated_depth < 0.0f || estimated_depth == INIT_DEPTH)
      continue;

    PointPtr point = nullptr;
    size_t start_frame = obs_frames.front();
    auto &feature_per_cam = feature_per_frame[start_frame];
    const std::vector<int> obs_cam_ids = feature_per_cam.getObsCamIds();
    if(obs_cam_ids.empty())
     continue;

    ++point_cnt;

    FrameBundlePtr &frame_bundle = states_[start_frame].frame_bundle;

    int cam_idx = obs_cam_ids.front();
    const FramePtr &frame = frame_bundle->at(cam_idx);
    const Eigen::Vector3d &pts = feature_per_cam.getObservation(cam_idx);

    Eigen::Vector3d pts_cam = pts * estimated_depth;
      
    Eigen::Vector3d pts_world = frame->T_world_cam() * pts_cam;
    point = std::make_shared<Point>(feature_id, pts_world);

    // std::cout << point->pos_.transpose() << std::endl;

    for(auto &frame_id : obs_frames)
    {
      FrameBundlePtr frame_bundle = states_[frame_id].frame_bundle;
      auto &feature_per_cam = feature_per_frame[frame_id];
      const std::vector<int> obs_cam_ids = feature_per_cam.getObsCamIds();

      for(auto &cam_idx : obs_cam_ids)
      {
        size_t ftr_idx = feature_per_cam.getObsFtrIdx(cam_idx);

        const FramePtr &frame = frame_bundle->at(cam_idx);
        frame->landmark_vec_[ftr_idx] = point;
        addLandmarkObs(frame->landmark_vec_[ftr_idx], frame, ftr_idx);
        frame_cnt[frame_id]++;

        if(cornerFeature(frame->type_vec_[ftr_idx]))
          frame->type_vec_[ftr_idx] = FeatureType::kCorner;
        else if(edgeletFeature(frame->type_vec_[ftr_idx]))
          frame->type_vec_[ftr_idx] = FeatureType::kEdgelet;
      }
    }
  }

  printf("Mono initialize: %d landmark were created\n", point_cnt);
  
  std::cout << "[frame num, point count]: ";
  for(int i = 0; i < WINDOW_SIZE + 1; ++i)
    std::cout << "[" << i << ", " << frame_cnt[i] << "], ";
  std::cout << std::endl;
}

const Eigen::Vector3d Estimator::getOdomVelScale()
{
  return odom_vel_scale_;
}

const Eigen::Vector3d Estimator::getOdomGyrScale()
{
  return odom_gyr_scale_;
}

const double Estimator::getTdOdom()
{
  return td_odom_;
}

const double Estimator::getTdImu()
{
  return td_imu_;
}

const bool Estimator::isBackendOK()
{
  return !failure_occur_;
}

void Estimator::optimizePose(const FrameBundlePtr &new_frames)
{
  int count = 0;
  std::vector<std::unordered_map<size_t, Eigen::Vector3d>> id_pts_maps;

  for(const FramePtr& frame : *new_frames)
  {
    std::unordered_map<size_t, Eigen::Vector3d> id_pts_map;

    for(size_t idx = 0; idx < numFeatures(frame); ++idx)
    {
      size_t feature_id = frame->track_id_vec_(idx);

      if(feature_per_ids_.count(feature_id) == 0)
        continue;

      int solve_flag = feature_per_ids_[feature_id].solve_flag;
      double estimated_depth = feature_per_ids_[feature_id].estimated_depth;
      if(solve_flag != 1 || estimated_depth == INIT_DEPTH)
        continue;

      auto &feature_per_frame = feature_per_ids_[feature_id].feature_per_frame;
      auto &obs_frames = feature_per_ids_[feature_id].obs_frames;
      int frame_id = obs_frames.back();
          
      FrameBundlePtr &frame_bundle = states_[frame_id].frame_bundle;
      
      const std::vector<int> obs_cam_ids = feature_per_frame[frame_id].getObsCamIds();

      if(obs_cam_ids.empty())
        continue;

      ++count;

      int cam_idx = obs_cam_ids.front();
      Transformation T_cam_world = frame_bundle->at(cam_idx)->T_f_w_;
      const Eigen::Vector3d &pts = feature_per_frame[frame_id].getObservation(cam_idx);

      Eigen::Vector3d pts_cam = pts * estimated_depth;
      Eigen::Vector3d pts_world = inverse(T_cam_world) * pts_cam;

      id_pts_map[feature_id] = pts_world;
    }
    id_pts_maps.emplace_back(id_pts_map);  
  }
  
  if(count < 100) 
    return;

  // std::cout << "optimize pose: point size: " << count << std::endl;

  #ifdef __MIVINS__
  optimize_pose_->Reset();
  #endif

  Transformation T_imu_world = getWorld2ImuTrans(new_frames);
  optimize_pose_->setTransformPrior(T_imu_world, 0.5);
  
  // Quaterniond R_imu_world = getRotation(T_imu_world);
  // optimize_pose_->setRotationPrior(R_imu_world, 0.0);
  // Transformation T_world_imu = getImu2WorldTrans(new_frames);
  // std::cout << "before run: \n" << T_world_imu << std::endl;
  optimize_pose_->run(new_frames, id_pts_maps);

  // Transformation T_world_imu = getImu2WorldTrans(new_frames);
  // std::cout << "after run: \n" << T_world_imu << std::endl;
}

// NOTE: not useful, for most time, it is not need to optimize the structure
void Estimator::optimizeStructure(
  const FrameBundlePtr &new_frames, 
  int max_n_pts, int max_iter)
{
  std::vector<size_t> feature_ids;

  for(const FramePtr& frame : *new_frames)
  {
    bool optimize_on_sphere = false;

    if (getCamType(frame) == Camera::Type::kOmni 
      || getCamType(frame) == Camera::Type::kMei)
      optimize_on_sphere = true;
    
    std::deque<PointPtr> pts;
    for (size_t i = 0; i < frame->num_features_; ++i)
    {
      if(i >= frame->landmark_vec_.size())
        continue;

      if (frame->landmark_vec_[i] == nullptr) // || isEdgelet(frame->type_vec_[i]))
        continue;

      size_t feature_id = frame->track_id_vec_(i);
      if(std::find(feature_ids.begin(), feature_ids.end(), feature_id) != feature_ids.end())
      {
        int solve_flag = feature_per_ids_[feature_id].solve_flag;
        double estimated_depth = feature_per_ids_[feature_id].estimated_depth;
        if(solve_flag == 1 && estimated_depth != INIT_DEPTH)
          continue;
      }

      if(frame->landmark_vec_[i]->obs_.size() < 2)
        continue;
      
      pts.push_back((frame->landmark_vec_[i]));
    }

    // std::cout << "pts size: " << pts.size() << "; "; 

    auto it_end = pts.end();
    if (max_n_pts > 0)
    {
      max_n_pts = std::min(static_cast<size_t>(max_n_pts), pts.size());
      std::nth_element(pts.begin(), pts.begin() + max_n_pts, pts.end(), 
              [](const PointPtr& lhs, const PointPtr& rhs)
      {
        // we favour points that have not been optimized in a while
        return (lhs->last_structure_optim_ < rhs->last_structure_optim_);
      });
      it_end = pts.begin() + max_n_pts;
    }

    int update_count = 0;
    for (const PointPtr& point : pts)
    {
      // point->optimize(max_iter, optimize_on_sphere);
      pointOptimize(point, max_iter, optimize_on_sphere);
      point->last_structure_optim_ = getFrameId(frame);

      size_t feature_id = getPointId(point); // point->id();
      
      if(std::find(feature_ids.begin(), feature_ids.end(), feature_id) != feature_ids.end())
      {
        FramePtr ref_frame = point->obs_.back().frame.lock();
        if(!ref_frame) continue;

        Eigen::Vector3d pts_world = point->pos();
        Eigen::Vector3d pts_cam = ref_frame->T_f_w_ * pts_world;
        double estimated_depth = USE_BEARING_VECTOR ? pts_cam.norm() : pts_cam.z();
        // if(pts_cam.z() < MIN_DEPTH || pts_cam.z() > MAX_DEPTH) continue;
        feature_per_ids_[feature_id].estimated_depth = estimated_depth;
        ++update_count;
      }
    }
    // std::cout << "update_count: " << update_count << "\n";
  }
}

void Estimator::createMotionDetector(const VinsMotionDetectorOptions& motion_detector_options)
{
  motion_detector_.reset(new VinsMotionDetector(motion_detector_options));
}

bool Estimator::motionDetect(const FrameBundlePtr& last_frames, 
                 const FrameBundlePtr& new_frames,
                 const ImuMeasurements& imu_measurements,
                 const OdomMeasurements& odom_measurements)
{
  if(!motion_detector_)
    return false;

  bool imu_motion_detector_stationary = false;
  bool odom_motion_detector_stationary = false;
  bool img_motion_detector_stationary = false;

  double sigma = 0;
  motion_detector_->setFrames(last_frames, new_frames);
  
  bool res = motion_detector_->isImageMoving(sigma);
  img_motion_detector_stationary = !res;
  

  if(imu_handler_)
  {
    int imu_rate = imu_handler_->imu_calib_.imu_rate;
    res = motion_detector_->isImuMoving(imu_rate, imu_measurements);
    imu_motion_detector_stationary = !res;
  }

  if(odom_handler_)
  {
    int odom_rate = odom_handler_->odom_calib_.odom_rate;
    res = motion_detector_->isOdomMoving(odom_rate, odom_measurements);
    odom_motion_detector_stationary = !res;
    
  }

  return imu_motion_detector_stationary 
    || odom_motion_detector_stationary 
    || img_motion_detector_stationary;
}

void Estimator::setMargedKF(const FrameBundlePtr &marged_frames)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  marged_frames_ = marged_frames;
}

void Estimator::getMargedKF(FrameBundlePtr &marged_frames)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  if(!marged_frames_) 
    return ;

  double marged_frames_ts = getTimestamp(marged_frames_);
  if(last_marged_ts_  == marged_frames_ts)
    return ;

  marged_frames = marged_frames_;
  last_marged_ts_ = marged_frames_ts;
}

void Estimator::valueInterpolation(
    const double ts1, const Eigen::Vector3d &val1, 
    const double ts2, const Eigen::Vector3d &val2,
    const double ts, Eigen::Vector3d &val)
{
  double scalar1 = (ts - ts1) / (ts2 - ts1);
  double scalar2 = 1.0 - scalar1;
  val = scalar2 * val1 + scalar1 * val2;
}

void Estimator::poseInterpolation(
    const double ts1, const Eigen::Quaterniond &q1, const Eigen::Vector3d &p1, 
    const double ts2, const Eigen::Quaterniond &q2, const Eigen::Vector3d &p2,
    const double ts, Eigen::Quaterniond &q, Eigen::Vector3d &p)
{
  p.setZero();
  q.setIdentity();

  double scalar1 = (ts - ts1) / (ts2 - ts1);
  q = quatSlerp(q1.normalized(), q2.normalized(), scalar1);
  
  double scalar2 = 1.0 - scalar1;
  p = scalar2 * p1 + scalar1 * p2;
}

Eigen::Quaterniond Estimator::quatSlerp(Eigen::Quaterniond qa, Eigen::Quaterniond qb, double scalar) 
{
  Eigen::Quaterniond qm;
  double cosHalfTheta = qa.w() * qb.w() + qa.x() * qb.x() + qa.y() * qb.y() + qa.z() * qb.z();

  if(cosHalfTheta < 0.0f)
  {
    qb.w() = -qb.w();
    qb.x() = -qb.x();
    qb.y() = -qb.y();
    qb.z() = -qb.z();
    cosHalfTheta = -cosHalfTheta;
  }

  if (abs(cosHalfTheta) >= 1.0)
  {
    qm.w() = qa.w();
    qm.x() = qa.x();
    qm.y() = qa.y();
    qm.z() = qa.z();
    return qm;
  }
  
  double halfTheta = acos(cosHalfTheta);
  double sinHalfTheta = sqrt(1.0 - cosHalfTheta*cosHalfTheta);
  if (fabs(sinHalfTheta) < 0.001)
  {
    qm.w() = (qa.w() * 0.5 + qb.w() * 0.5);
    qm.x() = (qa.x() * 0.5 + qb.x() * 0.5);
    qm.y() = (qa.y() * 0.5 + qb.y() * 0.5);
    qm.z() = (qa.z() * 0.5 + qb.z() * 0.5);
    return qm;
  }
  
  double ratioA = sin((1 - scalar) * halfTheta) / sinHalfTheta;
  double ratioB = sin(scalar * halfTheta) / sinHalfTheta; 
  
  qm.w() = (qa.w() * ratioA + qb.w() * ratioB);
  qm.x() = (qa.x() * ratioA + qb.x() * ratioB);
  qm.y() = (qa.y() * ratioA + qb.y() * ratioB);
  qm.z() = (qa.z() * ratioA + qb.z() * ratioB);
  return qm;
}

void Estimator::printPoses()
{
  for(int i = 0; i <= WINDOW_SIZE; ++i)
  {
    auto &frame_bundle = states_[i].frame_bundle;
    double ts = getTimestamp(frame_bundle);
    
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = states_[i].R;
    pose.block<3, 1>(0, 3) = states_[i].P;
    printf("frame count: %d, ts: %.9lf\n", i,  ts);
    std::cout << "pose: \n" << pose << std::endl;
  }
}

void Estimator::printPose(int frame_count)
{
  if(frame_count > WINDOW_SIZE)
    return;
  
  auto &frame_bundle = states_[frame_count].frame_bundle;
  double ts = getTimestamp(frame_bundle);
  
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose.block<3, 3>(0, 0) = states_[frame_count].R;
  pose.block<3, 1>(0, 3) = states_[frame_count].P;
  printf("frame count: %d, ts: %.9lf\n", frame_count,  ts);
  std::cout << "pose: \n" << pose << std::endl;
}

void Estimator::printPose(const FrameBundlePtr frame_bundle)
{
  double ts = getTimestamp(frame_bundle);
  
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  Transformation T = getImu2WorldTrans(frame_bundle);
  pose.block<3, 3>(0, 0) = getRotationMatrix(T);
  pose.block<3, 1>(0, 3) = getPosition(T);
  printf("frame count: %d, ts: %.9lf\n", frame_count_,  ts);
  std::cout << "pose: \n" << pose << std::endl;
}

void Estimator::printExtrinsic()
{
  if(ESTIMATE_CAM_EXTRINSIC)
  {
    assert(ric_.size() == tic_.size());
    for(int i = 0; i < (int)ric_.size(); ++i)
    {
      printf("T_imu_cam%d: \n", i);
      Eigen::Matrix4d T_I_C = Eigen::Matrix4d::Identity();
      T_I_C.block<3, 3>(0, 0) = ric_[i];
      T_I_C.block<3, 1>(0, 3) = tic_[i];
      std::cout << T_I_C << "\n\n";
    }
  }

  if(ESTIMATE_ODOM_EXTRINSIC)
  {
    printf("T_imu_odom:\n");
    Eigen::Matrix4d T_I_O = Eigen::Matrix4d::Identity();
    T_I_O.block<3, 3>(0, 0) = rio_;
    T_I_O.block<3, 1>(0, 3) = tio_;
    std::cout << T_I_O << "\n\n";
  }

  if(ESTIMATE_ODOM_TD)
    std::cout << "odom TD: " << td_odom_ << "\n";
  if(ESTIMATE_IMU_TD)
    std::cout << "cam TD: " << td_imu_ << "\n";
}
