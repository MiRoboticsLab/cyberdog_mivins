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

#include "include/mivins_process.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <image_transport/image_transport.hpp>
#include <string>
#include <vector>
#include <memory>
#include <map>
#include <queue>
#include <utility>

namespace mivins
{
using namespace std::chrono_literals;
template<typename T>
T readParam(rclcpp::Node::SharedPtr n, std::string name)
{
  T ans;
  n->declare_parameter<std::string>(name, "Unknown");
  if (n->get_parameter(name, ans)) {
    INFO_STREAM("[MIVINS][READPARAM]Loaded " << name << ": " << ans);
  } else {
    ERROR_STREAM("[MIVINS][READPARAM]Failed to load " << name);
    rclcpp::shutdown();
  }
  return ans;
}

template<typename T>
T param(
  rclcpp::Node::SharedPtr nh, const std::string & name, const T & defaultValue,
  bool silent = false)
{
  T v;
  if (!nh->has_parameter(name)) {
    nh->declare_parameter(name);
  }
  if (nh->get_parameter(name, v)) {
    if (!silent) {
      INFO_STREAM("[MIVINS][PARAM]Found parameter: " << name << ", value: " << v);
    }
    return v;
  }
  if (!silent) {
    WARN_STREAM(
      "[MIVINS][PARAM]Cannot find value for parameter: " << name << ", assigning default: " <<
        defaultValue);
  }
  return defaultValue;
}

template<typename T>
T readParam(nav2_util::LifecycleNode * n, std::string name)
{
  T ans;
  n->declare_parameter<std::string>(name, "Unknown");
  if (n->get_parameter(name, ans)) {
    INFO_STREAM("[MIVINS][READPARAM2]Loaded " << name << ": " << ans);
  } else {
    ERROR_STREAM("[MIVINS][READPARAM2]Failed to load " << name);
    rclcpp::shutdown();
  }
  return ans;
}
template<typename T>
T param(
  nav2_util::LifecycleNode * nh, const std::string & name, const T & defaultValue,
  bool silent = false)
{
  T v;
  if (!nh->has_parameter(name)) {
    nh->declare_parameter(name);
  }
  if (nh->get_parameter(name, v)) {
    if (!silent) {
      INFO_STREAM("[MIVINS][PARAM2]Found parameter: " << name << ", value: " << v);
    }
    return v;
  }
  if (!silent) {
    WARN_STREAM(
      "[MIVINS][PARAM2]Cannot find value for parameter: " << name << ", assigning default: " <<
        defaultValue);
  }
  return defaultValue;
}

MivinsProcess::MivinsProcess(
  rclcpp::Node::SharedPtr node_nh,
  const std::string calib_file,
  const std::string config_file)
: pnh_(node_nh)
{
  calib_file_ = calib_file;
  config_file_ = config_file;
  INFO_STREAM("[MIVINS][MIVINSPROCESS]calib_file_=" << calib_file_);

  FILE * fh_config = fopen(config_file_.c_str(), "r");
  if (fh_config == NULL) {
    INFO_STREAM("[MIVINS][MIVINSPROCESS]config_file path: " << config_file_);
    ERROR_STREAM("[MIVINS][MIVINSPROCESS]config_file dosen't exist; wrong config_file path");
    rclcpp::shutdown();
  }
  fclose(fh_config);
  FILE * fh_calib = fopen(calib_file_.c_str(), "r");
  if (fh_calib == NULL) {
    INFO_STREAM("[MIVINS][MIVINSPROCESS]calib file path: " << calib_file_);
    ERROR_STREAM("[MIVINS][MIVINSPROCESS]calib_file dosen't exist; wrong calib_file path");
    rclcpp::shutdown();
  }
  fclose(fh_calib);
  create_map_client_ = pnh_->create_client<std_srvs::srv::SetBool>("create_map_service");
  finish_map_client_ = pnh_->create_client<cyberdog_visions_interfaces::srv::FinishMap>(
    "finish_map_service");
  create_nav_client_ = pnh_->create_client<std_srvs::srv::SetBool>("start_nav_service");
  stop_nav_client_ = pnh_->create_client<std_srvs::srv::SetBool>("stop_nav_service");
  control_elevation_map_client_ = pnh_->create_client<std_srvs::srv::SetBool>(
    "elevation_mapping_step_monitor");

  MiVins_StartUp(config_file_, calib_file_);
  image_thread_ = nullptr;
  b_miloc_run_success_ = false;
  b_miloc_get_info_ = false;
  b_comeoutofserver_ = false;
}

void MivinsProcess::initMivins()
{
  WARN("[INITMIVINS]in initMivins !");
  start_map_ = false;
  finish_map_ = false;
  reloc_lastreq_time_ = 0;
  b_in_active_server_ = false;
  reloc_num_find_abnormal_ = 3;
  miloc_start_map = false;
  miloc_finish_map = false;
  end_flag = false;
  mivins_mode_ = LifeCycleMode::DEFAULT;
  start_elevation_map_ = false;
  finish_elevation_map_ = false;
  b_get_start_by_reloc_ = false;
  mivins_mode_stable_ = 0;
  while (!relocResponses_.empty()) {
    relocResponses_.pop();
  }
  img_lock.lock();
  while (!img_buffer.empty()) {
    img_buffer.pop();
  }
  img_lock.unlock();
  INFO_STREAM("[MIVINS][INITMIVINS]Img_buffer:" << img_buffer.size());
  std::string trace_dir = "/home/mi/";
  size_t cam_size = 3;
  b_miloc_run_success_ = false;
  b_miloc_get_info_ = false;
  b_comeoutofserver_ = false;
  visualizer_.reset(
    new Visualizer(pnh_, config_file_, trace_dir, cam_size));
}

void MivinsProcess::endMivins()
{
  MiVins_ShutDown();
}

void MivinsProcess::deleteImg()
{
  img_lock.lock();
  while (!img_buffer.empty()) {
    img_buffer.pop();
  }
  img_lock.unlock();
}

void MivinsProcess::savefile()
{
  MiVins_SaveFile();
}
void MivinsProcess::finishfile()
{
  MiVins_finish_SaveFile();
}

MivinsProcess::~MivinsProcess()
{
  INFO("[MIVINS][~MIVINSPROCESS]mivins delete process ...");
  if (image_thread_) {
    if (image_thread_->joinable()) {
      image_thread_->join();
      image_thread_ = nullptr;
    }
  }
  if (wait_reloc_thread_) {
    if (wait_reloc_thread_->joinable()) {
      wait_reloc_thread_->join();
      wait_reloc_thread_ = nullptr;
    }
  }
  if (stop_reloc_thread_) {
    if (stop_reloc_thread_->joinable()) {
      stop_reloc_thread_->join();
      stop_reloc_thread_ = nullptr;
    }
  }
  if (wait_map_thread_) {
    if (wait_map_thread_->joinable()) {
      wait_map_thread_->join();
      wait_map_thread_ = nullptr;
    }
  }
  if (stop_map_thread_) {
    if (stop_map_thread_->joinable()) {
      stop_map_thread_->join();
      stop_map_thread_ = nullptr;
    }
  }
  INFO("[MIVINS][~MIVINSPROCESS]mivins delete process complete! ");
}

void MivinsProcess::publishResults(
  const std::vector<cv::Mat> & images,
  const int64_t timestamp_nanoseconds)
{
  int stage = MiVins_GetStage();

  INFO_STREAM("[MIVINS][PUBLISHRESULTS]publishResults stage = " << stage);
  switch (stage) {
    case 2:
      {
        const std::string sensor_type = "cam";
        std::vector<std::vector<double>> last_cam_poses = MiVins_GetLastFramesPose(sensor_type);
        if (last_cam_poses.empty()) {
          WARN("[MIVINS][PUBLISHRESULTS]Warning: last_cam_poses is empty!");
          break;
        }
        if (last_cam_poses.size() > 0) {
          INFO_STREAM(
            "[MIVINS][PUBLISHRESULTS]publishResults last_cam_poses.size = " <<
              static_cast<int>(last_cam_poses.size()) <<
              " Pxyz = [ " << last_cam_poses[0][0] << "," <<
              last_cam_poses[0][1] << "," <<
              last_cam_poses[0][2] << " ]");
        }
        visualizer_->pubOdometry(last_cam_poses, timestamp_nanoseconds);
        double time_stamp = static_cast<double>(timestamp_nanoseconds * 1e-9);
        Eigen::Affine3d trans_vodom_baselink;

        const std::string sensor_odom = "odom";
        std::vector<std::vector<double>> last_odom_poses = MiVins_GetLastFramesPose(sensor_odom);
        Eigen::Quaterniond q_odom(last_odom_poses[0][6], last_odom_poses[0][3],
          last_odom_poses[0][4], last_odom_poses[0][5]);
        Eigen::Translation3d t_odom(last_odom_poses[0][0], last_odom_poses[0][1],
          last_odom_poses[0][2]);
        save_twb.precision(16);
        save_twb << time_stamp << " " << last_odom_poses[0][0] << " " << last_odom_poses[0][1] <<
          " " << last_odom_poses[0][2] << " " <<
          q_odom.x() << " " << q_odom.y() << " " << q_odom.z() << " " << q_odom.w() << std::endl;
        trans_vodom_baselink = (t_odom * q_odom);

        if (start_map_) {
          visualizer_->publishBaselinkToMapOdometry(time_stamp, trans_vodom_baselink, mivins_mode_);
          visualizer_->publishBaselinkToVodomTF(time_stamp, trans_vodom_baselink);
          Eigen::Affine3d trans_vodom2map;
          Eigen::Quaterniond q(1.0, 0.0, 0.0, 0.0);
          Eigen::Translation3d t(0.0, 0.0, 0.0);
          trans_vodom2map = (t * q);
          visualizer_->publishVodomToMapTF(time_stamp, trans_vodom2map);
        }

        if (mivins_mode_ == LifeCycleMode::NAVIGATION) {
          double reloc_time_interval = 3.5;
          if (visualizer_->init_reloc_) {
            reloc_time_interval = 3.0;
          }
          const std::string sensor_type_imu = "imu";
          std::vector<std::vector<double>> last_imu_pose =
            MiVins_GetLastFramesPose(sensor_type_imu);
          std::vector<double> imu_pose = last_imu_pose[0];
          visualizer_->pubIMUOdometry(time_stamp, imu_pose);
          double dtime = (timestamp_nanoseconds - reloc_lastreq_time_) / 1000000000.0;
          if (!visualizer_->init_reloc_) {
            reloc_frame_id_ = 0;
          }
          if (reloc_frame_id_ == 0 && dtime > reloc_time_interval) {
            auto request = std::make_shared<cyberdog_visions_interfaces::srv::Reloc::Request>();
            request->reloc_id = reloc_frame_id_;
            reloc_lastreq_time_ = timestamp_nanoseconds;
            auto result = reloc_client_->async_send_request(
              request, std::bind(
                &MivinsProcess::RelocCallback,
                this,
                std::placeholders::_1));
            INFO_STREAM(
              "[MIVINS][PUBLISHRESULTS]init reloc request success ------time: " << std::to_string(
                reloc_lastreq_time_ / 1000000000.0));
            reloc_frame_id_++;
            last_mivins_pose_ = imu_pose;
          } else if (reloc_frame_id_ != 0 && dtime > reloc_time_interval) {
            double distance_pub_reloc_ = sqrt(
              (imu_pose[0] - last_mivins_pose_[0]) * (imu_pose[0] - last_mivins_pose_[0]) +
              (imu_pose[1] - last_mivins_pose_[1]) * (imu_pose[1] - last_mivins_pose_[1]) +
              (imu_pose[2] - last_mivins_pose_[2]) * (imu_pose[2] - last_mivins_pose_[2]) );

            if (distance_pub_reloc_ > 0.5) {
              auto request = std::make_shared<cyberdog_visions_interfaces::srv::Reloc::Request>();
              request->reloc_id = reloc_frame_id_;
              reloc_lastreq_time_ = timestamp_nanoseconds;
              auto result = reloc_client_->async_send_request(
                request, std::bind(
                  &MivinsProcess::RelocCallback,
                  this,
                  std::placeholders::_1));
              INFO_STREAM(
                "[MIVINS][PUBLISHRESULTS]normal reloc request success!Time:" << std::to_string(
                  reloc_lastreq_time_ / 1000000000.0));
              reloc_frame_id_++;
              last_mivins_pose_ = imu_pose;
            }
          }
          std::vector<double> imu_to_camera_calib = MiVins_GetCameraImuCalib();

          std::vector<double> baselink_to_imu_calib = MiVins_GetImuBaselinkCalib();

          Eigen::Affine3d trans_map_vodom = visualizer_->publishVodomToMapTF(
            time_stamp, last_cam_poses[last_cam_poses.size() - 2],
            relocResponses_, imu_to_camera_calib, baselink_to_imu_calib);
          if (visualizer_->init_reloc_) {
            visualizer_->publishBaselinkToMapOdometry(
              time_stamp,
              trans_map_vodom * trans_vodom_baselink,
              mivins_mode_);
            visualizer_->publishBaselinkToVodomTF(time_stamp, trans_vodom_baselink);
          }
        }
        if (mivins_mode_ == LifeCycleMode::FOLLOWING) {
          visualizer_->publishBaselinkToVodomTF(time_stamp, trans_vodom_baselink);
        }
      }
  }
}
void MivinsProcess::publishResultsFollowing(
  const int64_t timestamp_nanoseconds)
{
  int stage = MiVins_GetStage();
  switch (stage) {
    case 2:
      {
        double time_stamp = static_cast<double>(timestamp_nanoseconds * 1e-9);
        Eigen::Affine3d trans_vodom_baselink;

        const std::string sensor_odom = "odom";
        std::vector<std::vector<double>> last_odom_poses = MiVins_GetLastFramesPose(sensor_odom);
        Eigen::Quaterniond q_odom(last_odom_poses[0][6], last_odom_poses[0][3],
          last_odom_poses[0][4], last_odom_poses[0][5]);
        Eigen::Translation3d t_odom(last_odom_poses[0][0], last_odom_poses[0][1],
          last_odom_poses[0][2]);
        trans_vodom_baselink = (t_odom * q_odom);

        if (start_map_) {
          visualizer_->publishBaselinkToMapOdometry(time_stamp, trans_vodom_baselink, mivins_mode_);
        }

        visualizer_->publishBaselinkToVodomTF(time_stamp, trans_vodom_baselink);
        visualizer_->publishVodomToMapTFStatic(time_stamp);
      }
  }
}

void MivinsProcess::monoCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  if (idle_) {
    return;
  }

  cv::Mat image;
  try {
    image = cv_bridge::toCvCopy(msg)->image;
  } catch (cv_bridge::Exception & e) {
    ERROR_STREAM("cv_bridge exception" << e.what());
  }

  std::vector<cv::Mat> images;
  images.push_back(image.clone());
}

void MivinsProcess::stereoCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr msg0,
  const sensor_msgs::msg::Image::ConstSharedPtr msg1)
{
  if (b_reset_) {
    INFO_MILLSECONDS(3000, "[MIVINS][STEREOCALLBACK] reset success!");
    sleep(3.0);
    return;
  }
  if (idle_) {
    return;
  }
  if (mivins_mode_ == LifeCycleMode::MAPPING || mivins_mode_ == LifeCycleMode::FOLLOWING) {
    if (!miloc_start_map) {
      return;
    }
    if (end_flag) {
      return;
    }
    if (miloc_finish_map) {
      INFO_STREAM("[MIVINS][STEREOCALLBACK]mapserver maping end !!!");
      end_flag = true;
      return;
    }
  } else if (mivins_mode_ == LifeCycleMode::NAVIGATION) {
    if (!b_get_start_by_reloc_) {
      INFO_STREAM("[MIVINS][STEREOCALLBACK]waiting for reloc start_info!");
      sleep(0.5);
      return;
    }
  } else {
    INFO_STREAM("[MIVINS][STEREOCALLBACK]waiting for activate!");
    return;
  }
  int64_t curr_t = rclcpp::Time(msg0->header.stamp).nanoseconds();

  cv::Mat img0, img1;
  try {
    img0 = cv_bridge::toCvShare(msg0, "mono8")->image;
    img1 = cv_bridge::toCvShare(msg1, "mono8")->image;
  } catch (cv_bridge::Exception & e) {
    ERROR_STREAM("[MIVINS][STEREOCALLBACK]cv_bridge exception:" << e.what());
  }
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
  clahe->apply(img0, img0);
  clahe->apply(img1, img1);
  ImgData img_data;
  img0.copyTo(img_data.img_left);
  img1.copyTo(img_data.img_right);

  img_data.time_stamp = curr_t;
  img_lock.lock();
  img_buffer.push(img_data);
  img_lock.unlock();
  last_time = img_data.time_stamp;
}

void MivinsProcess::stereoProcess()
{
  if (b_reset_) {
    sleep(1.0);
    INFO_MILLSECONDS(1000, "[MIVINS][STEREOPROCESS] reset success!");
    return;
  }
  while (true) {
    if (mivins_mode_ == LifeCycleMode::DEFAULT) {
      INFO_MILLSECONDS(1000, "[MIVINS][STEREOPROCESS] mivins: waiting for activate!");
      sleep(1.0);
      break;
    } else if (mivins_mode_ == LifeCycleMode::NAVIGATION) {
      if (b_get_start_by_reloc_) {
        break;
      } else {
        INFO_MILLSECONDS(1000, "[MIVINS][stereoProcess]waiting for reloc start_info!");
        sleep(1.0);
      }
    } else if (mivins_mode_ == LifeCycleMode::MAPPING || mivins_mode_ == LifeCycleMode::FOLLOWING) {
      if (miloc_start_map) {
        break;
      } else {
        INFO_MILLSECONDS(1000, "[MIVINS][stereoProcess]waiting for start_map activate!");
        sleep(1.0);
      }
    }
  }
  while (!b_reset_) {
    ImgData img_data;
    img_lock.lock();
    if (!img_buffer.empty()) {
      bool bRealTimeProcess = true;
      if (bRealTimeProcess) {
        while (!img_buffer.empty()) {
          img_data = img_buffer.front();
          img_buffer.pop();
        }
      } else {
        img_data = img_buffer.front();
        img_buffer.pop();
      }
    } else {
      img_lock.unlock();
      sleep(0.05);
      continue;
    }
    img_lock.unlock();

    cv::Mat img0, img1;
    img0 = img_data.img_left;
    img1 = img_data.img_right;
    int64_t time_stamp = img_data.time_stamp;
    std::vector<cv::Mat> images;
    std::map<int, cv::Mat> depths;
    images.emplace_back(img0);
    images.emplace_back(img1);
    MiVins_ImgInput(time_stamp, images, depths);
    if (normal_mode_ || mivins_mode_ == LifeCycleMode::MAPPING ||
      mivins_mode_ == LifeCycleMode::NAVIGATION)
    {
      publishResults(images, time_stamp);
    }
    b_process_status_ = true;
  }

  b_process_status_ = false;
}

void MivinsProcess::subscribeMapServer()
{
  app_create_map_service_ = pnh_->create_service<std_srvs::srv::SetBool>(
    "start_vins_mapping",
    std::bind(
      &MivinsProcess::CreateMapCallback,
      this, std::placeholders::_1, std::placeholders::_2));
  app_finish_mapname_service_ = pnh_->create_service<cyberdog_visions_interfaces::srv::FinishMap>(
    "stop_vins_mapping",
    std::bind(
      &MivinsProcess::FinishMapCallback,
      this, std::placeholders::_1, std::placeholders::_2));
}
void MivinsProcess::CreateMapCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (b_in_active_server_) {
    response->success = true;
    response->message = "app_start_location_service error: please do not duplicate request!";
    INFO_STREAM(
      "[MIVINS][CREATEMAPCALLBACK]app_start_mivins_map_service error:duplicate request!");
  } else {
    INFO_STREAM("[MIVINS][CREATEMAPCALLBACK]request->data:" << request->data);
    if (request->data) {
      start_map_ = true;
      response->success = true;
      response->message = "mivins Start to create map";
      activateMivins();
      b_in_active_server_ = true;
    } else {
      response->success = false;
      response->message = "app_finish_map_service error";
    }
  }
}

void MivinsProcess::FinishMapCallback(
  const std::shared_ptr<cyberdog_visions_interfaces::srv::FinishMap::Request> request,
  const std::shared_ptr<cyberdog_visions_interfaces::srv::FinishMap::Response> response)
{
  if (!b_in_active_server_) {
    response->success = true;
    response->message = "app_finish_map_service error: please do not duplicate request!";
    INFO_STREAM(
      "[MIVINS][CREATEMAPCALLBACK]app_finish_map_service error: duplicate request!");
  } else {
    miloc_finish = request->finish;
    miloc_map_name = request->map_name;
    finish_map_ = true;
    response->success = true;
    response->message = "mivins finish map";
    deactivateMivins();
    cleanupMivins();
    b_in_active_server_ = false;
  }
}

void MivinsProcess::subscribeNavigationServer()
{
  app_start_navigation_service_ = pnh_->create_service<std_srvs::srv::SetBool>(
    "start_vins_location",
    std::bind(
      &MivinsProcess::StartNavigationCallback,
      this, std::placeholders::_1, std::placeholders::_2));
  app_finish_navigation_service_ = pnh_->create_service<std_srvs::srv::SetBool>(
    "stop_vins_location",
    std::bind(
      &MivinsProcess::FinishNavigationCallback,
      this, std::placeholders::_1, std::placeholders::_2));
}

void MivinsProcess::StartNavigationCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (b_in_active_server_) {
    response->success = true;
    response->message = "app_start_location_service error: please do not duplicate request!";
    INFO_STREAM(
      "[MIVINS][CREATENAVIGATIONCALLBACK]app_start_location_service error:duplicate request!");
  } else {
    INFO_STREAM("request->data:" << request->data);
    if (request->data) {
      response->success = true;
      response->message = "mivins start location";
      activateMivins();
      b_in_active_server_ = true;
    } else {
      response->success = false;
      response->message = "app_start_location_service error";
    }
  }
}

void MivinsProcess::FinishNavigationCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (!b_in_active_server_) {
    response->success = true;
    response->message = "app_finish_location_service error: please do not duplicate request!";
    INFO_STREAM(
      "[MIVINS][FINISHNAVIGATIONCALLBACK]app_end_navlocation_service error:duplicate request!");
  } else {
    if (request->data) {
      response->success = true;
      response->message = "mivins finish location";
      deactivateMivins();
      cleanupMivins();
      b_in_active_server_ = false;
    } else {
      response->success = false;
      response->message = "app_finish_location_service error";
    }
  }
}
void MivinsProcess::activateMivins()
{
  INFO_STREAM("[MIVINS][ACTIVATEMIVINS]activateMivins....");
  b_reset_ = false;
  if (mivins_mode_ == LifeCycleMode::NAVIGATION) {
    if (mivins_mode_stable_ == 1) {
      MiVins_SetStableMode();
    }
  } else if (mivins_mode_ == LifeCycleMode::FOLLOWING) {
    miloc_start_map = true;
    MiVins_SetStableMode();
  }
  if (MiVins_GetImuStatus()) {
    subscribeImu();
  }
  if (MiVins_GetOdomStatus()) {
    subscribeOdom();
  }
  subscribeRobotStatus();
  subscribeImage();
}

void MivinsProcess::deactivateMivins()
{
  INFO_STREAM("[MIVINS][STOPMIVINS]In function_deactivatemivins.");
  b_reset_ = true;
  while (b_process_status_) {
    INFO_MILLSECONDS(1000, "[MIVINS][STOPMIVINS]Warning: Waiting for processing done");
    sleep(1);
  }
  INFO_STREAM("[MIVINS][STOPMIVINS]Processing done");

  if (mivins_mode_ == LifeCycleMode::MAPPING) {
    finish_map_ = true;
    createFinishMap();
    finishfile();
    save_twb.close();
  } else if (mivins_mode_ == LifeCycleMode::NAVIGATION) {
    createStopRelc();
    INFO_STREAM("[MIVINS][STOPMIVINS]Create STOPMIVINS Success.");
  } else if (mivins_mode_ == LifeCycleMode::FOLLOWING) {
    finish_map_ = true;
    miloc_finish_map = true;
    start_elevation_map_ = false;
    finish_elevation_map_ = false;
    normal_mode_ = false;
    last_publish_time_ = 0;
  }
  sub_imu.reset();
  INFO_STREAM("[MIVINS][STOPMIVINS]Reset SubImu Success.");
  sub_odom.reset();
  INFO_STREAM("[MIVINS][STOPMIVINS]Reset SubOdom Success.");
  sub_status.reset();
  INFO_STREAM("[MIVINS][STOPMIVINS]Reset SubStatus Success.");
  if (image_thread_ != nullptr) {
    INFO_STREAM(
      "[MIVINS][STOPMIVINS]Mivins_process kill result:   " <<
        pthread_cancel(image_thread_->native_handle()));
    image_thread_->detach();
    INFO_STREAM("[MIVINS][STOPMIVINS]Detach imagethread Success.");
  }

  temp_filter.disconnect();
  INFO_STREAM("[MIVINS][STOPMIVINS]Disconnect Filter Success.");
  sync_cam_.clear();
  INFO_STREAM("[MIVINS][STOPMIVINS]Reset SyncCam Success.");
  sync_subs_cam_.clear();
  INFO_STREAM("[MIVINS][STOPMIVINS]Reset SyncSubsCam Success.");
  deleteImg();
  INFO_STREAM("[MIVINS][STOPMIVINS]STOPMIVINS Success....");
}

void MivinsProcess::cleanupMivins()
{
  if (mivins_mode_ == LifeCycleMode::MAPPING) {
    INFO_STREAM("[MIVINS][CLEANUPMIVINS]1....");
    if (wait_map_thread_) {
      if (wait_map_thread_->joinable()) {
        wait_map_thread_->join();
        wait_map_thread_ = nullptr;
      }
    }
    INFO_STREAM("[MIVINS][CLEANUPMIVINS]2....");
    if (stop_map_thread_) {
      if (stop_map_thread_->joinable()) {
        stop_map_thread_->join();
        stop_map_thread_ = nullptr;
      }
    }

  } else if (mivins_mode_ == LifeCycleMode::NAVIGATION) {
    reloc_client_.reset();
    INFO_STREAM("[MIVINS][CLEANUPMIVINS]3....");
    if (wait_reloc_thread_) {
      if (wait_reloc_thread_->joinable()) {
        wait_reloc_thread_->join();
        wait_reloc_thread_ = nullptr;
      }
    }
    INFO_STREAM("[MIVINS][CLEANUPMIVINS]4....");
    if (stop_reloc_thread_) {
      if (stop_reloc_thread_->joinable()) {
        stop_reloc_thread_->join();
        stop_reloc_thread_ = nullptr;
      }
    }
  }
  visualizer_->visualizerReset();
  visualizer_.reset();
  endMivins();
  INFO_STREAM("[MIVINS][CLEANUPMIVINS]cleanupMivins....");
}

void MivinsProcess::milocCreatMapCallback(
  rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture result)
{
  if (result.get()) {
    INFO_STREAM("[MIVINS][MILOCCREATEMAPCALLBACK]milocCreatMapCallback...");
    INFO_STREAM(result.get()->success);
    INFO_STREAM(result.get()->message);
    miloc_start_map = true;
  } else {
    INFO_STREAM("[MIVINS][MILOCCREATEMAPCALLBACK]milocCreatMapCallback...Failed...");
  }
}

void MivinsProcess::milocFinishMapCallback(
  rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture result)
{
  if (result.get()) {
    INFO_STREAM("[MIVINS][MILOCFINISHMAPCALLBACK]milocFinishMapCallback...");
    INFO_STREAM("[MIVINS][MILOCFINISHMAPCALLBACK]" << result.get()->success);
    INFO_STREAM("[MIVINS][MILOCFINISHMAPCALLBACK]" << result.get()->message);
    miloc_finish_map = true;
  } else {
    INFO_STREAM("[MIVINS][MILOCFINISHMAPCALLBACK]milocFinishMapCallback...Failed...");
  }
}
void MivinsProcess::createWaitRelc()
{
  wait_reloc_thread_ =
    std::unique_ptr<std::thread>(new std::thread(&MivinsProcess::startNavigation, this));
}
void MivinsProcess::createStopRelc()
{
  stop_reloc_thread_ =
    std::unique_ptr<std::thread>(new std::thread(&MivinsProcess::stopNavigation, this));
}
void MivinsProcess::createWaitMap()
{
  wait_map_thread_ = std::unique_ptr<std::thread>(new std::thread(&MivinsProcess::createMap, this));
}
void MivinsProcess::createFinishMap()
{
  stop_map_thread_ = std::unique_ptr<std::thread>(new std::thread(&MivinsProcess::finishMap, this));
}

void MivinsProcess::startNavigation()
{
  int n = 0;
  while (!create_nav_client_->wait_for_service(1s)) {
    if (b_comeoutofserver_) {
      WARN("[MIVINS][STARTNAVIGATION]come out of start reloc return!");
      return;
    }
    if (!rclcpp::ok()) {
      WARN("[MIVINS][STARTNAVIGATION]Interrupted while waiting for the service.Exiting!");
      return;
    } else {
      INFO_STREAM("[MIVINS][STARTNAVIGATION]Service startNav not available,waiting again!");
      sleep(1.0);
    }
    n++;
    if (n > 5) {
      WARN_STREAM("[MIVINS][STARTNAVIGATION]Service not get!");
      return;
    }
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  using ServiceResponseFuture =
    rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      bool b_milocStartNav = future.get()->success;
      INFO_STREAM(
        "[MIVINS][STARTNAVIGATION]miloc:If startNavigation success:" << future.get()->success);
      INFO_STREAM(
        "[MIVINS][STARTNAVIGATION]miloc:startNavigation message:" <<
          future.get()->message);
      INFO_STREAM("[MIVINS][STARTNAVIGATION]Calling create_nav_service completed!");
      if (b_milocStartNav) {
        b_get_start_by_reloc_ = true;
        b_miloc_run_success_ = true;
      } else {
        b_get_start_by_reloc_ = false;
        b_miloc_run_success_ = false;
        ERROR("[MIVINS][STARTNAVIGATION]miloc:If startNavigation failed!");
      }
      b_miloc_get_info_ = true;
    };
  auto future_result = create_nav_client_->async_send_request(request, response_received_callback);
  INFO_STREAM("[MIVINS][STARTNAVIGATION]Calling create_nav_service completed---!");
}

void MivinsProcess::stopNavigation()
{
  INFO_STREAM("[MIVINS][STOPNAVIGATION]In stopNavigation...");
  int n = 0;
  while (!stop_nav_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      INFO_STREAM("[MIVINS][STOPNAVIGATION]Interrupted while waiting for the service. Exiting...");
      return;
    } else {
      WARN_STREAM("[MIVINS][STOPNAVIGATION]Service stopNavigation not available, waiting again...");
      sleep(1.0);
    }
    n++;
    if (n > 5) {
      WARN_STREAM("[MIVINS][ENDNAVIGATION]Service not get!");
      return;
    }
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  using ServiceResponseFuture =
    rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      INFO_STREAM("[MIVINS][STOPNAVIGATION]miloc:stopNav success:" << future.get()->success);
      INFO_STREAM("[MIVINS][STOPNAVIGATION]miloc:stopNav message:" << future.get()->message);
      INFO_STREAM("[MIVINS][STOPNAVIGATION]Calling stop_nav_service succeeded");
      b_get_start_by_reloc_ = false;
    };
  auto future_result = stop_nav_client_->async_send_request(request, response_received_callback);
  INFO_STREAM("[MIVINS][STOPNAVIGATION]stopNavigation complete...");
}
void MivinsProcess::createMap()
{
  INFO_STREAM("[MIVINS][CREATEMAP]start_map_:" << start_map_);
  int n = 0;
  while (!create_map_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      INFO_STREAM("[MIVINS][CREATEMAP]Interrupted while waiting for the service. Exiting...");
      return;
    } else {
      WARN_STREAM("[MIVINS][CREATEMAP]Service not available, waiting again...");
    }
    n++;
    if (n > 5) {
      WARN_STREAM("[MIVINS][CREATEMAP]Service not get!");
      return;
    }
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  using ServiceResponseFuture =
    rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      INFO_STREAM("[MIVINS][CREATEMAP]miloc:createMap success:" << future.get()->success);
      INFO_STREAM("[MIVINS][CREATEMAP]miloc:createMap message:" << future.get()->message);
      if (future.get()->success) {
        miloc_start_map = true;
        INFO_STREAM("[MIVINS][CREATEMAP]Calling create_map_service succeeded");
      }
    };
  auto future_result = create_map_client_->async_send_request(request, response_received_callback);
}

void MivinsProcess::finishMap()
{
  INFO_STREAM("[MIVINS][FINISHMAP]finish_map_:" << finish_map_);
  int n = 0;
  while (!finish_map_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      INFO_STREAM("[MIVINS][FINISHMAP]Interrupted while waiting for the service. Exiting...");
      return;
    } else {
      WARN_STREAM("[MIVINS][FINISHMAP]Service not available, waiting again...");
    }
    n++;
    if (n > 5) {
      WARN_STREAM("[MIVINS][finishMAP]Service not get!");
      return;
    }
  }
  auto request = std::make_shared<cyberdog_visions_interfaces::srv::FinishMap::Request>();
  request->finish = miloc_finish;
  request->map_name = miloc_map_name;

  using ServiceResponseFuture =
    rclcpp::Client<cyberdog_visions_interfaces::srv::FinishMap>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      INFO_STREAM("[MIVINS][FINISHMAP]miloc:finishMap success:" << future.get()->success);
      INFO_STREAM("[MIVINS][FINISHMAP]miloc:finishMap message:" << future.get()->message);
      INFO_STREAM("[MIVINS][FINISHMAP]Calling finish_map_service succeeded");
      miloc_finish_map = true;
    };
  auto future_result = finish_map_client_->async_send_request(request, response_received_callback);
}
void MivinsProcess::startElevationMap()
{
  if (start_elevation_map_) {
    return;
  }
  if (!control_elevation_map_client_->wait_for_service(1ms)) {
    if (!rclcpp::ok()) {
      INFO_MILLSECONDS(1000, "Interrupted while waiting for the service. Exiting...");
    } else {
      WARN_MILLSECONDS(1000, "Time out when waiting for elevation mapping service. Skipping...");
    }
    return;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  using ServiceResponseFuture =
    rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      INFO_STREAM("[MIVINS][STARTELEVATIONMAP]Calling elevation_mapping_start succeeded");
      start_elevation_map_ = true;
    };
  auto future_result = control_elevation_map_client_->async_send_request(
    request,
    response_received_callback);
}

void MivinsProcess::finishElevationMap()
{
  if (finish_elevation_map_) {
    return;
  }

  if (!control_elevation_map_client_->wait_for_service(3ms)) {
    if (!rclcpp::ok()) {
      INFO_STREAM(
        "[MIVINS][FINISHELEVATIONMAP]Interrupted while waiting for elev_map_serv.Exiting!");
    } else {
      WARN_STREAM(
        "[MIVINS][FINISHELEVATIONMAP]TimeOut when waiting for elev_map_serv.Skipping!");
    }
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;
  using ServiceResponseFuture =
    rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      INFO_STREAM("[MIVINS][FINISHELEVATIONMAP]Calling elevation_mapping_stop succeeded");
      finish_elevation_map_ = true;
    };
  auto future_result = control_elevation_map_client_->async_send_request(
    request,
    response_received_callback);
}
void MivinsProcess::robotStatusCallback(
  const protocol::msg::MotionStatus::ConstSharedPtr status_msg)
{
  if (b_reset_) {
    INFO_MILLSECONDS(1000, "[STATUSCALLBACK] reset success!");
    sleep(0.5);
    return;
  }
  const int motion_id = status_msg->motion_id;
  MiVins_StatusInput(motion_id);
}
void MivinsProcess::subscribeRobotStatus()
{
  std::string status_topic = status_topic_;
  INFO_STREAM("[MIVINS][SUBSCRIBEROBOTSTATUS]status_topic:" << status_topic);
  rclcpp::SensorDataQoS sub_qos;
  sub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  sub_status = pnh_->create_subscription<protocol::msg::MotionStatus>(
    status_topic, sub_qos,
    std::bind(&mivins::MivinsProcess::robotStatusCallback, this, std::placeholders::_1)
  );
}
void MivinsProcess::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (b_reset_) {
    INFO_MILLSECONDS(1000, "[IMUCALLBACK] reset success!");
    sleep(0.5);
    return;
  }
  if (mivins_mode_ == LifeCycleMode::MAPPING || mivins_mode_ == LifeCycleMode::FOLLOWING) {
    if (end_flag) {
      return;
    }
  } else if (mivins_mode_ == LifeCycleMode::NAVIGATION) {
    if (!b_get_start_by_reloc_) {
      INFO_MILLSECONDS(1000, "[MIVINS][IMUCALLBACK]Waiting for reloc start_info!");
      sleep(0.5);
      return;
    }
  } else {
    INFO_MILLSECONDS(1000, "[MIVINS][IMUCALLBACK]Waiting for activate!");
    return;
  }
  const Eigen::Vector3d omega_imu(
    msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  const Eigen::Vector3d lin_acc_imu(
    msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  MiVins_ImuInput(rclcpp::Time(msg->header.stamp).seconds(), lin_acc_imu, omega_imu);
}

void MivinsProcess::subscribeImu()
{
  std::string imu_topic = imu_topic_;
  INFO_STREAM("[MIVINS][SUBSCRIBEIMU]-------imu_topic:" << imu_topic);

  if (!normal_mode_ && mivins_mode_ == LifeCycleMode::FOLLOWING) {
    return;
  }

  rclcpp::SensorDataQoS sub_qos;
  sub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  sub_imu =
    pnh_->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, sub_qos,
    std::bind(&mivins::MivinsProcess::imuCallback, this, std::placeholders::_1));
}

void MivinsProcess::subscribeOdom()
{
  std::string odom_topic = odom_topic_;
  INFO_STREAM("[MIVINS][SUBSCRIBEODOM]-------odom_topic----  " << odom_topic);
  rclcpp::SensorDataQoS sub_qos;
  sub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  sub_odom =
    pnh_->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, sub_qos,
    std::bind(&mivins::MivinsProcess::odomCallback, this, std::placeholders::_1));
}

void MivinsProcess::subscribeReloc()
{
  reloc_client_ = pnh_->create_client<cyberdog_visions_interfaces::srv::Reloc>("reloc_service");
}

void MivinsProcess::RelocCallback(
  rclcpp::Client<cyberdog_visions_interfaces::srv::Reloc>::SharedFuture result)
{
  auto response = result.get();
  float confidence_threshold = 0.5;
  float reloc_pose_find_time_threshold = 0.1;
  RelocResponse reloc_response;
  reloc_response.reloc_id = response->reloc_id;
  reloc_response.status = response->reply.status;
  reloc_response.is_verified = response->is_verified;
  reloc_response.time = rclcpp::Time(response->pose.header.stamp).nanoseconds();
  reloc_response.confidence = response->confidence;
  INFO_STREAM(
    "[MIVINS][RELOCCALLBACK]reloc_cb response.time:" << std::to_string(reloc_response.time));
  reloc_response.x = response->pose.pose.position.x;
  reloc_response.y = response->pose.pose.position.y;
  reloc_response.z = response->pose.pose.position.z;
  reloc_response.rx = response->pose.pose.orientation.x;
  reloc_response.ry = response->pose.pose.orientation.y;
  reloc_response.rz = response->pose.pose.orientation.z;
  reloc_response.rw = response->pose.pose.orientation.w;
  INFO_STREAM(
    "[MIVINS][RELOCCALLBACK]reloc verified:" << reloc_response.is_verified << " status:" <<
      reloc_response.status << " reloc_id:" << reloc_response.reloc_id);

  INFO_STREAM("[MIVINS][RELOCCALLBACK]reloc confidence_threshold:" << response->confidence);
  if (reloc_response.status == 1 || reloc_response.status == 2) {
    if (reloc_response.confidence >= confidence_threshold) {
      std::queue<RelocResponse> relocResponses_temp = relocResponses_;
      bool bRepeat = false;
      while (relocResponses_temp.size() > 0) {
        if (reloc_response.reloc_id == relocResponses_temp.front().reloc_id) {
          bRepeat = true;
          INFO_STREAM("[MIVINS][RELOCCALLBACK]reloc repeat get!");
          break;
        }
        relocResponses_temp.pop();
      }
      if (!bRepeat) {
        double reloc_time = rclcpp::Time(reloc_response.time).nanoseconds() / 1000000000.0;
        bool b_findpose = false;
        bool b_baddata = false;
        std::vector<double> imu_to_camera_calib = MiVins_GetCameraImuCalib();
        std::vector<double> baselink_to_imu_calib = MiVins_GetImuBaselinkCalib();
        Eigen::Quaterniond q_baselink_to_imu(baselink_to_imu_calib[6], baselink_to_imu_calib[3],
          baselink_to_imu_calib[4], baselink_to_imu_calib[5]);
        Eigen::Translation3d t_baselink_to_imu(baselink_to_imu_calib[0], baselink_to_imu_calib[1],
          baselink_to_imu_calib[2]);
        Eigen::Quaterniond q_imu_to_cam(imu_to_camera_calib[6], imu_to_camera_calib[3],
          imu_to_camera_calib[4], imu_to_camera_calib[5]);
        Eigen::Translation3d t_imu_to_cam(imu_to_camera_calib[0], imu_to_camera_calib[1],
          imu_to_camera_calib[2]);
        Eigen::Quaterniond q_reloc(reloc_response.rw,
          reloc_response.rx,
          reloc_response.ry,
          reloc_response.rz);
        Eigen::Translation3d t_reloc(reloc_response.x,
          reloc_response.y,
          reloc_response.z);
        for (int posen = visualizer_->path.poses.size() - 1; posen >= 0; posen--) {
          double odom_time =
            rclcpp::Time(visualizer_->path.poses[posen].header.stamp).nanoseconds() /
            1000000000.0;
          if (abs(odom_time - reloc_time) < reloc_pose_find_time_threshold) {
            Eigen::Quaterniond q_vodom(visualizer_->path.poses[posen].pose.orientation.w,
              visualizer_->path.poses[posen].pose.orientation.x,
              visualizer_->path.poses[posen].pose.orientation.y,
              visualizer_->path.poses[posen].pose.orientation.z);
            Eigen::Translation3d t_vodom(visualizer_->path.poses[posen].pose.position.x,
              visualizer_->path.poses[posen].pose.position.y,
              visualizer_->path.poses[posen].pose.position.z);
            Eigen::Affine3d uselastrelocinfo_baselink_to_map = visualizer_->vodom_to_map_ *
              (t_vodom * q_vodom) * (t_imu_to_cam * q_imu_to_cam) *
              (t_baselink_to_imu * q_baselink_to_imu);
            Eigen::Matrix4d uselastrelocinfo_baselink_to_map_matrix =
              uselastrelocinfo_baselink_to_map.matrix();
            double last_x = uselastrelocinfo_baselink_to_map_matrix(0, 3);
            double last_y = uselastrelocinfo_baselink_to_map_matrix(1, 3);
            double last_z = uselastrelocinfo_baselink_to_map_matrix(2, 3);
            double distance_reloc = sqrt(
              (reloc_response.x - last_x) * (reloc_response.x - last_x) +
              (reloc_response.y - last_y) * (reloc_response.y - last_y) +
              (reloc_response.z - last_z) * (reloc_response.z - last_z));
            if (distance_reloc > 2.0) {
              b_baddata = true;
            }
            b_findpose = true;
            break;
          }
        }
        if (reloc_response.reloc_id == 0) {
          b_baddata = false;
        }
        if (b_findpose && !b_baddata) {
          relocResponses_.push(reloc_response);
          INFO_STREAM(
            "[MIVINS][RELOCCALLBACK]reloc push into queue sucess!!! " <<
              relocResponses_.size() );
          std::vector<double> trans_test;
          trans_test.push_back(response->pose.pose.position.x);
          trans_test.push_back(response->pose.pose.position.y);
          trans_test.push_back(response->pose.pose.position.z);
          trans_test.push_back(response->pose.pose.orientation.x);
          trans_test.push_back(response->pose.pose.orientation.y);
          trans_test.push_back(response->pose.pose.orientation.z);
          trans_test.push_back(response->pose.pose.orientation.w);

          visualizer_->publishBaselinkToMapTest(
            rclcpp::Time(
              response->pose.header.stamp).nanoseconds() / 1000000000.0, trans_test);
        } else {
          WARN_STREAM("[MIVINS][RELOCCALLBACK]reloc can not find pose info ,time threshold !");
        }
      }
      INFO_STREAM(
        "[MIVINS][RELOCCALLBACK]reloc push response  sucess! queue size: " <<
          relocResponses_.size()  );
    }
  }

  while (relocResponses_.size() > reloc_num_find_abnormal_) {
    relocResponses_.pop();
    INFO_STREAM("[MIVINS][RELOCCALLBACK]reloc pop ---size: " << relocResponses_.size()   );
  }
  INFO_STREAM("[MIVINS][RELOCCALLBACK]reloc response call back finished!");
}
void MivinsProcess::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
{
  if (b_reset_) {
    return;
  }
  if (mivins_mode_ == LifeCycleMode::MAPPING || mivins_mode_ == LifeCycleMode::FOLLOWING) {
    if (end_flag) {
      return;
    }
  } else if (mivins_mode_ == LifeCycleMode::NAVIGATION) {
    if (!b_get_start_by_reloc_) {
      INFO_MILLSECONDS(1000, "[MIVINS][ODOMCALLBACK]waiting for reloc start_info!");
      sleep(0.5);
      return;
    }
  } else {
    INFO_MILLSECONDS(1000, "[MIVINS][ODOMCALLBACK]waiting for activate!");
    sleep(1.0);
    return;
  }
  double ts = rclcpp::Time(odom_msg->header.stamp).seconds();
  const Eigen::Vector3d position(odom_msg->pose.pose.position.x,
    odom_msg->pose.pose.position.y,
    odom_msg->pose.pose.position.z);

  const Eigen::Quaterniond orientation(odom_msg->pose.pose.orientation.w,
    odom_msg->pose.pose.orientation.x,
    odom_msg->pose.pose.orientation.y,
    odom_msg->pose.pose.orientation.z);

  const Eigen::Vector3d linear_velocity(odom_msg->twist.twist.linear.x,
    odom_msg->twist.twist.linear.y,
    odom_msg->twist.twist.linear.z);

  const Eigen::Vector3d angular_velocity(odom_msg->twist.twist.angular.x,
    odom_msg->twist.twist.angular.y,
    odom_msg->twist.twist.angular.z);

  MiVins_OdomInput(ts, orientation, position, linear_velocity, angular_velocity);

  if (!normal_mode_ && mivins_mode_ == LifeCycleMode::FOLLOWING) {
    int64_t timestamp_nanoseconds = rclcpp::Time(odom_msg->header.stamp).nanoseconds();
    publishResultsFollowing(timestamp_nanoseconds);
  }
}
void MivinsProcess::subscribeImage()
{
  int pipeline_type_ = MiVins_GetPipelineType();
  INFO_STREAM(
    "[MIVINS][SUBSCRIBEIMAGE]MivinsProcess::subscribeImage() pipeline_type_ = " <<
      pipeline_type_);
  if (!normal_mode_ && mivins_mode_ == LifeCycleMode::FOLLOWING) {
    return;
  }
  if (pipeline_type_ == 2) {
    std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr,
      const sensor_msgs::msg::Image::ConstSharedPtr)> cb =
      std::bind(
      &mivins::MivinsProcess::stereoCallback, this, std::placeholders::_1,
      std::placeholders::_2);

    std::string cam0_topic = cam0_topic_;
    std::string cam1_topic = cam1_topic_;
    INFO_STREAM(
      "[MIVINS][SUBSCRIBEIMAGE] cam0_topic:" << cam0_topic << " cam0_topic:" <<
        cam1_topic);
    auto sub_image0 = std::make_unique<ImageSyncSub>(
      pnh_, cam0_topic,
      rclcpp::SensorDataQoS().get_rmw_qos_profile());
    auto sub_image1 = std::make_unique<ImageSyncSub>(
      pnh_, cam1_topic,
      rclcpp::SensorDataQoS().get_rmw_qos_profile());
    auto sync = std::make_unique<ImageSync>(
      ImageSyncPolicy(10), *sub_image0,
      *sub_image1);
    temp_filter = sync->registerCallback(
      std::bind(cb, std::placeholders::_1, std::placeholders::_2));
    sync_cam_.push_back(std::move(sync));
    sync_subs_cam_.push_back(std::move(sub_image0));
    sync_subs_cam_.push_back(std::move(sub_image1));
    image_thread_ = std::unique_ptr<std::thread>(
      new std::thread(&MivinsProcess::stereoProcess, this));
  } else if (pipeline_type_ == 4) {
  }
}
}  // namespace mivins
