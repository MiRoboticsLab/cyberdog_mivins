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

#ifndef MIVINS_PROCESS_H_
#define MIVINS_PROCESS_H_
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/time.h>
#include <tf2_ros/async_buffer_interface.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <mivins_api.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <thread>
#include <queue>
#include <fstream>
#include <memory>
#include <vector>
#include <string>
#include <map>
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "cyberdog_visions_interfaces/srv/reloc.hpp"
#include "cyberdog_visions_interfaces/srv/finish_map.hpp"
#include "protocol/msg/motion_status.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "./visualizer.h"
namespace mivins
{
  enum LifeCycleMode
  {
    DEFAULT = 0,
    MAPPING = 1,
    NAVIGATION = 2,
    FOLLOWING = 3
  };

  enum class PipelineType
  {
    kMono,
    kStereo,
    kArray,
    kRgbdFisheye,
    kNone
  };

  typedef struct ImgData_
  {
    ImgData_() {
    }
    ~ImgData_() {
    }
    cv::Mat img_front;
    cv::Mat img_left;
    cv::Mat img_right;
    cv::Mat img_depth;
    int64_t time_stamp;
  } ImgData;

class MivinsProcess
  {
public:
    typedef message_filters::Subscriber < sensor_msgs::msg::Image > ImageSyncSub;
    typedef message_filters::sync_policies::ApproximateTime < sensor_msgs::msg::Image,
      sensor_msgs::msg::Image > ImageSyncPolicy;
    typedef message_filters::Synchronizer < ImageSyncPolicy > ImageSync;

    rclcpp::Node::SharedPtr pnh_;
    PipelineType pipeline_type_;

    std::unique_ptr < std::thread > image_thread_;
    std::unique_ptr < std::thread > wait_reloc_thread_;
    std::unique_ptr < std::thread > stop_reloc_thread_;

    std::unique_ptr < std::thread > wait_map_thread_;
    std::unique_ptr < std::thread > stop_map_thread_;

    std::queue < ImgData > img_buffer;
    std::mutex img_lock;

    std::shared_ptr < Visualizer > visualizer_;

    std::vector < std::unique_ptr < ImageSyncSub >> sync_subs_cam_;
    std::vector < std::unique_ptr < ImageSync >> sync_cam_;
    rclcpp::Subscription < sensor_msgs::msg::Imu > ::ConstSharedPtr sub_imu;
    rclcpp::Subscription < nav_msgs::msg::Odometry > ::ConstSharedPtr sub_odom;
    rclcpp::Subscription < protocol::msg::MotionStatus > ::ConstSharedPtr sub_status;

    int mivins_mode_;
    rclcpp::Service < std_srvs::srv::SetBool > ::SharedPtr app_create_map_service_;
    rclcpp::Service < cyberdog_visions_interfaces::srv::FinishMap >
    ::SharedPtr app_finish_mapname_service_;
    rclcpp::Service < std_srvs::srv::SetBool > ::SharedPtr app_start_navigation_service_;
    rclcpp::Service < std_srvs::srv::SetBool > ::SharedPtr app_finish_navigation_service_;
    rclcpp::Client < std_srvs::srv::SetBool > ::SharedPtr create_map_client_;
    rclcpp::Client < cyberdog_visions_interfaces::srv::FinishMap > ::SharedPtr finish_map_client_;
    bool start_map_;
    bool finish_map_;
    bool miloc_start_map;
    bool miloc_finish_map;
    bool end_flag;
    bool miloc_finish;
    std::string miloc_map_name;

    rclcpp::Client < cyberdog_visions_interfaces::srv::Reloc > ::SharedPtr reloc_client_;
    rclcpp::Client < std_srvs::srv::SetBool > ::SharedPtr create_nav_client_;
    rclcpp::Client < std_srvs::srv::SetBool > ::SharedPtr stop_nav_client_;
    bool b_get_start_by_reloc_;

    rclcpp::Client < std_srvs::srv::SetBool > ::SharedPtr control_elevation_map_client_;
    bool start_elevation_map_;
    bool finish_elevation_map_;

    bool normal_mode_ = false;

    bool quit_ = false;
    bool idle_ = false;
    bool automatic_reinitialization_ = false;
    bool set_initial_attitude_from_gravity_ = true;

    int64_t last_time = 0;
    float lG[256 * 256];
    float * lvignetteMapInv;
    float rG[256 * 256];
    float * rvignetteMapInv;
    std::map < int64_t, float > ltimes;
    std::map < int64_t, float > rtimes;
    std::ofstream savet_image;

    int64_t reloc_frame_id_ = 0;
    std::queue < RelocResponse > relocResponses_;
    int64_t reloc_lastreq_time_;
    int64_t last_publish_time_ = 0;
    int reloc_num_find_abnormal_;
    std::vector < double > last_mivins_pose_;
    bool b_reset_ = false;
    bool b_process_status_ = false;
    std::string cam0_topic_;
    std::string cam1_topic_;
    std::string imu_topic_;
    std::string odom_topic_;
    std::string status_topic_;
    std::string calib_file_;
    std::string config_file_;
    message_filters::Connection temp_filter;
    int mivins_mode_stable_;
    bool b_miloc_run_success_;
    bool b_miloc_get_info_;
    bool b_comeoutofserver_;
    bool b_in_active_server_;
    std::ofstream save_twb;

public:
    MivinsProcess(
      rclcpp::Node::SharedPtr node_nh,
      const std::string calib_file,
      const std::string config_file);

    virtual ~MivinsProcess();

    void endMivins();
    void deleteImg();
    void initMivins();
    void savefile();
    void finishfile();

    void publishResults(
      const std::vector < cv::Mat > & images,
      const int64_t timestamp_nanoseconds);
    void publishResultsFollowing(
      const int64_t timestamp_nanoseconds);
    void monoCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void stereoCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr msg0,
      const sensor_msgs::msg::Image::ConstSharedPtr msg1);

    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg);
    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
    void RelocCallback(
      rclcpp::Client < cyberdog_visions_interfaces::srv::Reloc > ::SharedFuture result);

    void stereoProcess();

    void FinishMapCallback(
      const std::shared_ptr < cyberdog_visions_interfaces::srv::FinishMap::Request > request,
      const std::shared_ptr < cyberdog_visions_interfaces::srv::FinishMap::Response > response);
    void CreateMapCallback(
      const std::shared_ptr < std_srvs::srv::SetBool::Request > request,
      const std::shared_ptr < std_srvs::srv::SetBool::Response > response);
    void StartNavigationCallback(
      const std::shared_ptr < std_srvs::srv::SetBool::Request > request,
      const std::shared_ptr < std_srvs::srv::SetBool::Response > response);
    void FinishNavigationCallback(
      const std::shared_ptr < std_srvs::srv::SetBool::Request > request,
      const std::shared_ptr < std_srvs::srv::SetBool::Response > response);
    void milocCreatMapCallback(rclcpp::Client < std_srvs::srv::SetBool > ::SharedFuture result);
    void milocFinishMapCallback(rclcpp::Client < std_srvs::srv::SetBool > ::SharedFuture result);
    void createMap();
    void finishMap();
    void createWaitMap();
    void createFinishMap();
    void activateMivins();
    void deactivateMivins();
    void cleanupMivins();

    void startNavigation();
    void stopNavigation();
    void createWaitRelc();
    void createStopRelc();

    void startElevationMap();
    void finishElevationMap();

    void subscribeRobotStatus();
    void robotStatusCallback(const protocol::msg::MotionStatus::ConstSharedPtr status_msg);

    void subscribeImu();
    void subscribeOdom();
    void subscribeImage();
    void subscribeReloc();
    void subscribeMapServer();
    void subscribeNavigationServer();

    void loadTimes(std::string file, int id_camera);
  };
}  // namespace mivins
#endif  // MIVINS_PROCESS_H_
