#pragma once

#include <thread>

#include <ros/ros.h>
#include <std_msgs/String.h> // user-input
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <vector>
#include <Eigen/Core>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <mivins_api.h>

#include "visualizer.h"

namespace mivins
{

    /// SVO Interface
    class SvoProcess
    {
    public:
        // ROS subscription and publishing.
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        ros::Subscriber sub_remote_key_;
        std::string remote_key_topic_;
        std::string remote_input_;
        std::unique_ptr<std::thread> imu_thread_;
        std::unique_ptr<std::thread> odom_thread_;
        std::unique_ptr<std::thread> image_thread_;

        std::shared_ptr<Visualizer> visualizer_;
        //std::shared_ptr<mivins::SvoInterface> svo_interface_;

        std::string calib_file_;
        std::string config_file_;

        // Parameters
        bool set_initial_attitude_from_gravity_ = true;

        // System state.
        bool quit_ = false;
        bool idle_ = false;
        bool automatic_reinitialization_ = false;

        SvoProcess(const ros::NodeHandle &nh,
                   const ros::NodeHandle &private_nh);

        virtual ~SvoProcess();

        // Processing
        void processImageBundle(
            const std::vector<cv::Mat> &images,
            const std::map<int, cv::Mat> &depths,
            int64_t timestamp_nanoseconds);

        bool setImuPrior(const int64_t timestamp_nanoseconds);
        bool setImuPrior_3dof(const int64_t timestamp_nanoseconds);

        void publishResults(
            const std::vector<cv::Mat> &images,
            const int64_t timestamp_nanoseconds);

        // Subscription and callbacks
        void rgbdCallback(const sensor_msgs::ImageConstPtr &msg_image,
                          const sensor_msgs::ImageConstPtr &msg_depth);

        void monoCallback(const sensor_msgs::ImageConstPtr &msg);

        void stereoCallback(
            const sensor_msgs::ImageConstPtr &msg0,
            const sensor_msgs::ImageConstPtr &msg1);

        void tripleWithStereoCallback(
            const sensor_msgs::ImageConstPtr &msg0,
            const sensor_msgs::ImageConstPtr &msg1,
            const sensor_msgs::ImageConstPtr &msg2);

        void tripleWithDepthCallback(
            const sensor_msgs::ImageConstPtr &msg_front,
            const sensor_msgs::ImageConstPtr &msg_left,
            const sensor_msgs::ImageConstPtr &msg_right,
            const sensor_msgs::ImageConstPtr &msg_depth);

        void imuCallback(const sensor_msgs::ImuConstPtr &imu_msg);
        void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg);
        void inputKeyCallback(const std_msgs::StringConstPtr &key_input);

        void subscribeImu();
        void subscribeOdom();
        void subscribeImage();
        void subscribeRemoteKey();

        void imuLoop();
        void odomLoop();
        void rgbdLoop();
        void monoLoop();
        void stereoLoop();
        void tripleWithStereoLoop();
        void tripleWithDepthLoop();
    };

} // namespace mivins
