// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).

#include "visualizer.h"
// #include <output_helper.h>
// #include <svo/param.h>

#include <deque>
#include <chrono>
#include <algorithm>
#include <iostream>
#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/tf.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

// #include <svo_msgs/dense_input.h>
// #include <svo_msgs/dense_input_with_features.h>
// #include <svo_msgs/Info.h>

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <yaml-cpp/yaml.h>
#pragma diagnostic pop

// namespace
// {
// typedef std::chrono::high_resolution_clock Clock;
// typedef std::chrono::time_point<Clock> TimePoint;
// typedef std::chrono::nanoseconds Nanoseconds;
// typedef std::chrono::seconds Seconds;

// static double getCurrentTime()
// {
//   return static_cast<double>(
//         std::chrono::duration_cast<Nanoseconds>(Clock::now()-TimePoint())
//         .count())*1e-9;
// }

// void publishLineList(
//     ros::Publisher& pub,
//     const std::vector<Eigen::Matrix<float, 1, 6>>& links,
//     const std::string& ns, const Eigen::Vector3f& color, const double scale,
//     const double alpha = 1.0)
// {
//   visualization_msgs::Marker marker;
//   marker.id = 0;
//   marker.ns = ns;
//   marker.header.frame_id = mivins::Visualizer::kWorldFrame;
//   marker.header.stamp = ros::Time::now();
//   marker.type = visualization_msgs::Marker::LINE_LIST;
//   marker.action = visualization_msgs::Marker::ADD;
//   marker.points.reserve(links.size());
//   for (size_t i = 0; i < links.size(); i++)
//   {
//     geometry_msgs::Point point;
//     point.x = static_cast<double>(links[i](0, 0));
//     point.y = static_cast<double>(links[i](0, 1));
//     point.z = static_cast<double>(links[i](0, 2));
//     marker.points.push_back(point);
//     point.x = static_cast<double>(links[i](0, 3));
//     point.y = static_cast<double>(links[i](0, 4));
//     point.z = static_cast<double>(links[i](0, 5));
//     marker.points.push_back(point);
//   }
//   marker.scale.x = 0.015 * scale;
//   marker.scale.y = 0.015 * scale;
//   marker.scale.z = 0.015 * scale;
//   marker.color.a = alpha;
//   marker.color.r = color(0);
//   marker.color.g = color(1);
//   marker.color.b = color(2);
//   pub.publish(marker);
// }

// void publishStringsAtPositions(
//     ros::Publisher& pub,
//     const std::vector<std::string>& strings,
//     const std::vector<Eigen::Vector3d>& positions)
// {
//   visualization_msgs::MarkerArray ma;
//   CHECK_EQ(strings.size(), positions.size());

//   for (size_t i = 0; i < positions.size(); i++)
//   {
//     visualization_msgs::Marker marker;
//     marker.header.frame_id = mivins::Visualizer::kWorldFrame;
//     marker.header.stamp = ros::Time::now();
//     marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//     marker.action = visualization_msgs::Marker::ADD;
//     marker.id = i;

//     marker.pose.position.x = positions[i].x();
//     marker.pose.position.y = positions[i].y();
//     marker.pose.position.z = positions[i].z();

//     marker.text = strings[i];
//     marker.scale.z = 0.1;
//     marker.color.a = 1.0;
//     marker.color.r = 0.0;
//     marker.color.g = 0.0;
//     marker.color.b = 0.0;
//     ma.markers.emplace_back(marker);
//   }
//   pub.publish(ma);
// }

// }

namespace mivins
{
    template <typename T>
    T getParam(const YAML::Node &config, const std::string &name, const T &default_value)
    {
        try
        {
            T value = config[name].as<T>();
            return value;
        }
        catch (...)
        {
            return default_value;
        }

        return default_value;
    }
    //std::string Visualizer::kWorldFrame = std::string("world");

    Visualizer::Visualizer(const ros::NodeHandle &nh_private,
                           const std::string config_file,
                           const std::string trace_dir,
                           const size_t n_cameras)
        : pnh_(nh_private), trace_dir_(trace_dir), pc_(new PointCloud)
    {
        YAML::Node config = YAML::LoadFile(config_file);
        img_pub_level_ = getParam<int>(config, "publish_img_pyr_level", 0);
        img_pub_nth_ = getParam<int>(config, "publish_every_nth_img", 1);
        dense_pub_nth_ = getParam<int>(config, "publish_every_nth_dense_input", 1);
        viz_caption_str_ = getParam<bool>(config, "publish_image_caption_str", false);

        publish_world_in_cam_frame_ =
            getParam<bool>(config, "publish_world_in_cam_frame", true);
        publish_map_every_frame_ =
            getParam<bool>(config, "publish_map_every_frame", false);
        publish_points_display_time_ = ros::Duration(
            getParam<double>(config, "publish_point_display_time", 0));
        publish_seeds_ = getParam<bool>(config, "publish_seeds", true);
        publish_seeds_uncertainty_ =
            getParam<bool>(config, "publish_seeds_uncertainty", false);
        publish_active_keyframes_ = getParam<bool>(config, "publish_active_kfs", false);
        trace_pointcloud_ = getParam<bool>(config, "trace_pointcloud", false);
        vis_scale_ = getParam<double>(config, "publish_marker_scale", 1.2);

        // Init ROS Marker Publishers
        pub_frames_ = pnh_.advertise<visualization_msgs::Marker>("keyframes", 10);
        pub_points_ = pnh_.advertise<visualization_msgs::Marker>("points", 10000);
        pub_imu_pose_ =
            pnh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_imu", 10);
        //pub_info_ = pnh_.advertise<svo_msgs::Info>("info", 10);
        pub_markers_ = pnh_.advertise<visualization_msgs::Marker>("markers", 100);
        pub_pc_ = pnh_.advertise<PointCloud>("pointcloud", 1);
        pub_dense_.resize(n_cameras);
        pub_images_.resize(n_cameras);
        pub_cam_poses_.resize(n_cameras);
        image_transport::ImageTransport it(pnh_);
        for (size_t i = 0; i < n_cameras; ++i)
        {
            // pub_dense_.at(i) = pnh_.advertise<svo_msgs::DenseInputWithFeatures>(
            //     "dense_input/" + std::to_string(i), 2);
            pub_images_.at(i) = it.advertise("image/" + std::to_string(i), 10);
            pub_cam_poses_.at(i) = pnh_.advertise<geometry_msgs::PoseStamped>(
                "pose_cam/" + std::to_string(i), 10);
        }

        pub_odometry = pnh_.advertise<nav_msgs::Odometry>("odometry/front", 1000);
        pub_odometryleft = pnh_.advertise<nav_msgs::Odometry>("odometry/left", 1000);
        pub_odometryright = pnh_.advertise<nav_msgs::Odometry>("odometry/right", 1000);
        // #ifdef SVO_LOOP_CLOSING
        // pose_graph_map_.clear();
        // pose_graph_map_.header.frame_id = kWorldFrame;
        // pub_loop_closure_ =
        //     pnh_.advertise<visualization_msgs::Marker>("loop_closures", 10);
        // pub_pose_graph_ = pnh_.advertise<PointCloud>("pose_graph", 10);
        // pub_pose_graph_map_ = pnh_.advertise<PointCloud>("pose_graph_pointcloud", 10);
        // #endif
    }

    // void Visualizer::publishSvoInfo(const int64_t timestamp_nanoseconds, const int stage,
    //           const int track_quality, const size_t last_obs_num, const double last_process_time)
    // {
    //   ++trace_id_;

    //   if (pub_info_.getNumSubscribers() == 0)
    //     return;

    //   svo_msgs::Info msg_info;
    //   msg_info.header.frame_id = "cam";
    //   msg_info.header.seq = trace_id_;
    //   msg_info.header.stamp = ros::Time().fromNSec(timestamp_nanoseconds);
    //   msg_info.processing_time = last_process_time;
    //   msg_info.stage = stage;
    //   msg_info.tracking_quality = track_quality;
    //   msg_info.num_matches = last_obs_num;
    //   pub_info_.publish(msg_info);
    // }

    // void Visualizer::publishImuPose(const Transformation& T_world_imu,
    //                                 const Eigen::Matrix<double, 6, 6> Covariance,
    //                                 const int64_t timestamp_nanoseconds)
    // {
    //   if (pub_imu_pose_.getNumSubscribers() == 0)
    //     return;
    //   VLOG(100) << "Publish IMU Pose";

    //   Eigen::Quaterniond q = T_world_imu.GetRotation().ToImplementation();
    //   Eigen::Vector3d p = T_world_imu.GetPosition();
    //   geometry_msgs::PoseWithCovarianceStampedPtr msg_pose(
    //       new geometry_msgs::PoseWithCovarianceStamped);
    //   msg_pose->header.seq = trace_id_;
    //   msg_pose->header.stamp = ros::Time().fromNSec(timestamp_nanoseconds);
    //   msg_pose->header.frame_id = mivins::Visualizer::kWorldFrame;
    //   msg_pose->pose.pose.position.x = p[0];
    //   msg_pose->pose.pose.position.y = p[1];
    //   msg_pose->pose.pose.position.z = p[2];
    //   msg_pose->pose.pose.orientation.x = q.x();
    //   msg_pose->pose.pose.orientation.y = q.y();
    //   msg_pose->pose.pose.orientation.z = q.z();
    //   msg_pose->pose.pose.orientation.w = q.w();
    //   for (size_t i = 0; i < 36; ++i)
    //     msg_pose->pose.covariance[i] = Covariance(i % 6, i / 6);
    //   pub_imu_pose_.publish(msg_pose);
    // }

    void Visualizer::publishCameraPoses(std::vector<std::vector<double>> cam_poses,
                                        const int64_t timestamp_nanoseconds)
    {
        if (cam_poses.empty())
            return;

        // output_helper::publishTfTransform(cam_poses[0].inverse(),
        //     ros::Time().fromNSec(timestamp_nanoseconds), "cam_pos", kWorldFrame, br_);

        for (size_t i = 0; i < cam_poses.size(); ++i)
        {
            if (pub_cam_poses_.at(i).getNumSubscribers() == 0)
                return;
            //VLOG(100) << "Publish camera pose " << i;

            //Eigen::Quaterniond q = cam_poses[i].GetRotation().ToImplementation();
            //Eigen::Vector3d p = cam_poses[i].GetPosition();
            geometry_msgs::PoseStampedPtr msg_pose(new geometry_msgs::PoseStamped);
            msg_pose->header.seq = trace_id_;
            msg_pose->header.stamp = ros::Time().fromNSec(timestamp_nanoseconds);
            msg_pose->header.frame_id = "cam" + std::to_string(i);
            msg_pose->pose.position.x = cam_poses[i][0];
            msg_pose->pose.position.y = cam_poses[i][1];
            msg_pose->pose.position.z = cam_poses[i][2];
            msg_pose->pose.orientation.x = cam_poses[i][3];
            msg_pose->pose.orientation.y = cam_poses[i][4];
            msg_pose->pose.orientation.z = cam_poses[i][5];
            msg_pose->pose.orientation.w = cam_poses[i][6];
            pub_cam_poses_.at(i).publish(msg_pose);
        }
    }

    // void Visualizer::publishBundleFeatureTracks(const FrameBundlePtr frames_ref,
    //                                             const FrameBundlePtr frames_cur,
    //                                             int64_t timestamp)
    // {
    //   if (trace_id_ % img_pub_nth_ != 0 || !frames_ref)
    //     return;
    //   VLOG(100) << "Publish bundle feature tracks.";

    //   for (size_t i = 0; i < frames_ref->size(); ++i)
    //   {
    //     std::vector<std::pair<size_t, size_t>> matches_ref_cur;
    //     feature_tracker_tools::getFeatureMatches(
    //         *frames_ref->at(i), *frames_cur->at(i), &matches_ref_cur);
    //     publishFeatureTracks(frames_ref->at(i)->px_vec_, frames_cur->at(i)->px_vec_,
    //                          matches_ref_cur, frames_cur->at(i)->img_pyr_,
    //                          img_pub_level_, timestamp, i);
    //   }
    // }

    void Visualizer::publishOdometry(std::vector<std::vector<double>> cam_poses,
                                     const int64_t timestamp_nanoseconds)
    {

        if (cam_poses.empty())
            return;
        for (size_t i = 0; i < cam_poses.size(); ++i)
        {

            nav_msgs::Odometry odometry;
            odometry.header.stamp = ros::Time().fromNSec(timestamp_nanoseconds);
            odometry.header.frame_id = "world";
            odometry.child_frame_id = "world";

            odometry.pose.pose.position.x = cam_poses[i][0];
            odometry.pose.pose.position.y = cam_poses[i][1];
            odometry.pose.pose.position.z = cam_poses[i][2]; //set zero
            odometry.pose.pose.orientation.x = cam_poses[i][3];
            odometry.pose.pose.orientation.y = cam_poses[i][4];
            odometry.pose.pose.orientation.z = cam_poses[i][5];
            odometry.pose.pose.orientation.w = cam_poses[i][6];
            odometry.twist.twist.linear.x = 0;
            odometry.twist.twist.linear.y = 0;
            odometry.twist.twist.linear.z = 0;

            if (i == 0)
            {
                pub_odometry.publish(odometry);
            }
            else if (i == 1)
            {
                pub_odometryleft.publish(odometry);
            }
            else if (i == 2)
            {
                pub_odometryright.publish(odometry);
            }
        }
    }

    // void Visualizer::publishFeatureTracks(
    //     const Eigen::MatrixXd& px_ref, const Eigen::MatrixXd& px_cur,
    //     const std::vector<std::pair<size_t, size_t>>& matches_ref_cur,
    //     const std::vector<cv::Mat>& img_pyr, const int& level, const uint64_t timestamp,
    //     const size_t frame_index)
    // {
    //   if (pub_images_.at(frame_index).getNumSubscribers() == 0)
    //     return;
    //   VLOG(100) << "Publish feature tracks.";
    //   const int scale = (1 << level);
    //   cv::Mat img_rgb(img_pyr[level].size(), CV_8UC3);
    //   cv::cvtColor(img_pyr[level], img_rgb, cv::COLOR_GRAY2RGB);
    //   for (size_t i = 0; i < matches_ref_cur.size(); ++i)
    //   {
    //     size_t i_ref = matches_ref_cur[i].first;
    //     size_t i_cur = matches_ref_cur[i].second;
    //     cv::line(img_rgb,
    //              cv::Point2f(px_cur(0, i_cur) / scale, px_cur(1, i_cur) / scale),
    //              cv::Point2f(px_ref(0, i_ref) / scale, px_ref(1, i_ref) / scale),
    //              cv::Scalar(0, 255, 0), 2);
    //   }
    //   writeCaptionStr(img_rgb);
    //   cv_bridge::CvImage img_msg;
    //   img_msg.header.frame_id = "cam";
    //   img_msg.header.seq = trace_id_;
    //   img_msg.header.stamp = ros::Time().fromNSec(timestamp);
    //   img_msg.image = img_rgb;
    //   img_msg.encoding = sensor_msgs::image_encodings::BGR8;
    //   pub_images_.at(frame_index).publish(img_msg.toImageMsg());
    // }

    // void Visualizer::publishImages(const int cam_idx,
    //                                const std::vector<cv::Mat>& img_pyr,
    //                                const int64_t timestamp_nanoseconds)
    // {
    //   if (trace_id_ % img_pub_nth_ != 0)
    //     return;
    //   VLOG(100) << "Publish images.";

    //   if (pub_images_.at(cam_idx).getNumSubscribers() == 0)
    //     return;

    //   cv_bridge::CvImage img_msg;
    //   img_msg.header.stamp = ros::Time().fromNSec(timestamp_nanoseconds);
    //   img_msg.header.frame_id = "cam" + std::to_string(cam_idx);
    //   img_msg.image = img_pyr.at(img_pub_level_);
    //   img_msg.encoding = sensor_msgs::image_encodings::MONO8;
    //   pub_images_.at(cam_idx).publish(img_msg.toImageMsg());
    // }

    // void Visualizer::publishImagesWithFeatures(int cam_idx,
    //                                     const int64_t timestamp,
    //                                     const std::vector<cv::Mat> &img_pyr,
    //                                     const Eigen::MatrixXd &feat_px,
    //                                     const Eigen::MatrixXd &feat_grad,
    //                                     const Eigen::MatrixXi &feat_status,
    //                                     const bool draw_boundary)
    // {
    //   if (trace_id_ % img_pub_nth_ != 0)
    //     return;

    //   if (pub_images_.at(cam_idx).getNumSubscribers() == 0)
    //     return;
    //   VLOG(100) << "Publish image with features " << cam_idx;

    //   cv::Mat img_rgb;
    //   DrawFeatures(img_pyr, feat_px, feat_grad, feat_status, true, img_pub_level_, &img_rgb);
    //   if (draw_boundary)
    //   {
    //     cv::rectangle(img_rgb, cv::Point2f(0.0, 0.0),
    //                   cv::Point2f(img_rgb.cols, img_rgb.rows),
    //                   cv::Scalar(0, 255, 0), 6);
    //   }
    //   writeCaptionStr(img_rgb);
    //   cv_bridge::CvImage img_msg;
    //   img_msg.header.frame_id = "cam";
    //   img_msg.header.seq = trace_id_;
    //   img_msg.header.stamp = ros::Time().fromNSec(timestamp);
    //   img_msg.image = img_rgb;
    //   img_msg.encoding = sensor_msgs::image_encodings::BGR8;
    //   pub_images_.at(cam_idx).publish(img_msg.toImageMsg());
    // }

    // void Visualizer::visualizeHexacopter(const Transformation& T_frame_world,
    //                                      const uint64_t timestamp)
    // {
    //   if (pub_frames_.getNumSubscribers() > 0)
    //   {
    //     output_helper::publishCameraMarker(pub_frames_, "cam_pos", "cams",
    //                                            ros::Time().fromNSec(timestamp), 1,
    //                                            0, 0.8, Eigen::Vector3d(0., 0., 1.));
    //   }
    // }

    // void Visualizer::visualizeQuadrocopter(const Transformation& T_frame_world,
    //                                        const uint64_t timestamp)
    // {
    //   output_helper::publishTfTransform(T_frame_world,
    //                                         ros::Time().fromNSec(timestamp),
    //                                         "cam_pos", kWorldFrame, br_);

    //   if (pub_frames_.getNumSubscribers() > 0)
    //   {
    //     output_helper::publishQuadrocopterMarkers(
    //         pub_frames_, "cam_pos", "cams", ros::Time().fromNSec(timestamp), 1, 0,
    //         0.8, Eigen::Vector3d(0., 0., 1.));
    //   }
    // }

    // void Visualizer::visualizeMarkers(const std::vector<Transformation> &last_frame_poses,
    //   const std::vector<Transformation> &active_kf_poses,
    //   std::unordered_map<int, Transformation> &close_kf_poses,
    //   const std::vector<Eigen::Vector4d> &map_region_pts,
    //   const std::vector<Eigen::Vector3d> map_seeds,
    //   const std::vector<Eigen::Vector3d> map_seeds_uncertainty,
    //   const std::unordered_map<int, Eigen::Vector4d> &close_kf_all_pts,
    //   const int64_t timestamp, const bool is_kf)
    // {
    //   visualizeHexacopter(last_frame_poses[0].inverse(), timestamp);
    //   publishTrajectoryPoint(last_frame_poses[0].GetPosition(), timestamp, trace_id_);
    //   if (is_kf || publish_map_every_frame_)
    //   {
    //     publishMapRegion(map_region_pts, close_kf_poses, close_kf_all_pts);
    //   }

    //   if (publish_seeds_)
    //     publishSeeds(map_seeds);
    //   if (publish_seeds_uncertainty_)
    //     publishSeedsUncertainty(map_seeds_uncertainty);
    //   if (publish_active_keyframes_)
    //   {
    //     publishActiveKeyframes(active_kf_poses);
    //   }
    // }

    // void Visualizer::publishTrajectoryPoint(const Eigen::Vector3d& pos_in_vision,
    //                                         const uint64_t timestamp, const int id)
    // {
    //   if (pub_points_.getNumSubscribers() > 0)
    //   {
    //     VLOG(100) << "Publish trajectory point.";
    //     output_helper::publishPointMarker(
    //         pub_points_, pos_in_vision, "trajectory",
    //         ros::Time().fromNSec(timestamp), id, 0,
    //         0.5 * trajectory_marker_scale_ * vis_scale_, Eigen::Vector3d(0., 0., 0.5));
    //   }
    // }

    // void Visualizer::publishSeeds(std::vector<Eigen::Vector3d> map_seeds)
    // {
    //   VLOG(100) << "Publish seeds.";
    //   double marker_scale = seed_marker_scale_ * vis_scale_;
    //   visualization_msgs::Marker m;
    //   m.header.frame_id = kWorldFrame;
    //   m.header.stamp = ros::Time();
    //   m.ns = "seeds";
    //   m.id = 0;
    //   m.type = visualization_msgs::Marker::POINTS;
    //   m.action = 0;  // add/modify
    //   m.scale.x = marker_scale;
    //   m.scale.y = marker_scale;
    //   m.scale.z = marker_scale;
    //   m.color.a = 1.0;
    //   m.color.r = 1.0;
    //   m.color.g = 0.0;
    //   m.color.b = 0.0;
    //   m.pose.orientation.x = 0.0;
    //   m.pose.orientation.y = 0.0;
    //   m.pose.orientation.z = 0.0;
    //   m.pose.orientation.w = 1.0;
    //   m.points.reserve(1000);

    //   for(size_t i = 0; i < map_seeds.size(); ++i)
    //   {
    //     const Eigen::Vector3d xyz = map_seeds[i];
    //     geometry_msgs::Point p;
    //     p.x = xyz.x();
    //     p.y = xyz.y();
    //     p.z = xyz.z();
    //     m.points.push_back(p);
    //   }

    //   pub_points_.publish(m);
    // }

    // void Visualizer::publishSeedsUncertainty(const std::vector<Eigen::Vector3d> &map_seeds_uncertainty)
    // {
    //   VLOG(100) << "Publish seed uncertainty.";
    //   double marker_scale = seed_uncertainty_marker_scale_ * vis_scale_;
    //   visualization_msgs::Marker msg_variance;
    //   msg_variance.header.frame_id = kWorldFrame;
    //   msg_variance.header.stamp = ros::Time();
    //   msg_variance.ns = "seeds_variance";
    //   msg_variance.id = 0;
    //   msg_variance.type = visualization_msgs::Marker::LINE_LIST;
    //   msg_variance.action = 0;  // add/modify
    //   msg_variance.scale.x = marker_scale;
    //   msg_variance.scale.y = marker_scale;
    //   msg_variance.scale.z = marker_scale;
    //   msg_variance.color.a = 1.0;
    //   msg_variance.color.r = 1.0;
    //   msg_variance.color.g = 0.0;
    //   msg_variance.color.b = 0.0;
    //   msg_variance.points.reserve(1000);

    //   size_t size = map_seeds_uncertainty.size() / 2;
    //   for(size_t i = 0; i < size; ++i)
    //   {
    //     Eigen::Vector3d p1 = map_seeds_uncertainty[ 2 * i];
    //     Eigen::Vector3d p2 = map_seeds_uncertainty[ 2 * i + 1];

    //     geometry_msgs::Point msg_point;
    //     msg_point.x = p1.x();
    //     msg_point.y = p1.y();
    //     msg_point.z = p1.z();
    //     msg_variance.points.push_back(msg_point);
    //     msg_point.x = p2.x();
    //     msg_point.y = p2.y();
    //     msg_point.z = p2.z();
    //     msg_variance.points.push_back(msg_point);
    //   }

    //   pub_points_.publish(msg_variance);
    // }

    // void Visualizer::publishMapRegion(const std::vector<Eigen::Vector4d> &map_regions,
    //                         const std::unordered_map<int, Transformation> &close_kf_poses,
    //                         const std::unordered_map<int, Eigen::Vector4d> &close_kf_all_pts)
    // {
    //   VLOG(100) << "Publish map region.";
    //   uint64_t ts = getCurrentTime();

    //   if (pub_pc_.getNumSubscribers() > 0)
    //   {
    //     pc_->header.frame_id = kWorldFrame;

    //     pcl_conversions::toPCL(ros::Time::now(), pc_->header.stamp);
    //     pc_->clear();
    //     PointType p;

    //     for (size_t i = 0; i < map_regions.size(); ++i)
    //     {
    //       std::cout << "publishMapRegion: " << i << std::endl;
    //       p.x = map_regions[i](0);
    //       p.y = map_regions[i](1);
    //       p.z = map_regions[i](2);
    //       p.intensity = (int)map_regions[i](3);
    //       pc_->push_back(p);
    //     }

    //     VLOG(100) << "Publish pointcloud of size " << pc_->size();
    //     pub_pc_.publish(pc_);
    //   }

    //   if (pub_points_.getNumSubscribers() > 0)
    //   {
    //     publishKeyframeWithPoints(close_kf_poses, close_kf_all_pts, point_marker_scale_);
    //   }
    // }

    // void Visualizer::publishKeyframeWithPoints(
    //     const std::unordered_map<int, Transformation> &cam_poses,
    //     const std::unordered_map<int, Eigen::Vector4d> &id_pts,
    //     const double marker_scale)
    // {
    //   // publish keyframe
    //   for(auto &it : cam_poses)
    //   {
    //     int frame_id = it.first;
    //     Transformation cam_pose = it.second;
    //     Transformation T_world_cam(cam_pose);
    //     output_helper::publishFrameMarker(
    //         pub_frames_, T_world_cam.GetRotationMatrix(), T_world_cam.GetPosition(),
    //         "kfs", ros::Time::now(), frame_id * 10, 0, marker_scale * 2.0);
    //   }

    //   for(auto &it : id_pts)
    //   {
    //     int id = it.first;
    //     Eigen::Vector4d pts = it.second;
    //     Eigen::Vector3d xyz_world = pts.head<3>();
    //     int type = (int)pts(3);

    //     if (type == 1) // edge
    //     {
    //       output_helper::publishPointMarker(
    //           pub_points_, xyz_world, "pts", ros::Time::now(), id, 0,
    //           marker_scale * vis_scale_, Eigen::Vector3d(0, 0.6, 0),
    //           publish_points_display_time_);
    //     }
    //     else           // corner
    //     {
    //       output_helper::publishPointMarker(
    //           pub_points_, xyz_world, "pts", ros::Time::now(), id, 0,
    //           marker_scale * vis_scale_, Eigen::Vector3d(1, 0, 1),
    //           publish_points_display_time_);
    //     }

    //     if (trace_pointcloud_)
    //     {
    //       if (!ofs_pointcloud_.is_open())
    //         ofs_pointcloud_.open(trace_dir_ + "/pointcloud.txt");
    //       ofs_pointcloud_ << xyz_world.x() << " " << xyz_world.y() << " "
    //                       << xyz_world.z() << std::endl;
    //     }
    //   }
    // }

    // void Visualizer::publishActiveKeyframes(const std::vector<Transformation>& active_kf_poses)
    // {
    //   const std::string ns("active_kfs");

    //   // visualize active keyframes as links
    //   if (active_kf_poses.size() < 1)
    //   {
    //     return;
    //   }
    //   for (size_t i = 0; i < active_kf_poses.size() - 1; i++)
    //   {
    //     output_helper::publishLineMarker(
    //         pub_frames_, active_kf_poses[i].GetPosition(),
    //         active_kf_poses[i + 1].GetPosition(), ns,
    //         ros::Time::now(), i, 0,  // add the marker
    //         0.8 * trajectory_marker_scale_ * vis_scale_,
    //         Eigen::Vector3d(.0, .0, 0.5));
    //   }
    // }

    // void Visualizer::exportToDense(
    //     const uint64_t ts,
    //     const std::vector<int> frame_id,
    //     const std::vector<cv::Mat> &imgs,
    //     const std::vector<Transformation> &frame_poses,
    //     const std::vector<Eigen::MatrixXd> &pts_worlds)
    // {
    //   VLOG(100) << "Publish dense input.";
    //   // for (size_t cam_index = 0; cam_index < frame_bundle->size(); ++cam_index)
    //   for(size_t cam_index = 0; cam_index < imgs.size(); ++cam_index)
    //   {
    //     if (dense_pub_nth_ > 0 && trace_id_ % dense_pub_nth_ == 0 &&
    //         pub_dense_.at(cam_index).getNumSubscribers() > 0)
    //     {
    //       uint64_t timestamp = ts;
    //       cv::Mat img = imgs[cam_index];
    //       Transformation frame_pose = frame_poses[cam_index];

    //       svo_msgs::DenseInputWithFeatures msg;
    //       msg.header.stamp = ros::Time().fromNSec(timestamp);
    //       msg.header.frame_id = mivins::Visualizer::kWorldFrame;
    //       msg.frame_id = frame_id[cam_index];

    //       cv_bridge::CvImage img_msg;
    //       img_msg.header.stamp = msg.header.stamp;
    //       img_msg.header.frame_id = "camera";

    //       if (!img.empty() && img.channels() == 3)
    //       {
    //         img_msg.image = img;
    //         img_msg.encoding = sensor_msgs::image_encodings::BGR8;
    //       }
    //       else if(!img.empty() && img.channels() == 1)
    //       {
    //         img_msg.image = img;
    //         img_msg.encoding = sensor_msgs::image_encodings::MONO8;
    //       }
    //       msg.image = *img_msg.toImageMsg();

    //       double min_z = std::numeric_limits<double>::max();
    //       double max_z = std::numeric_limits<double>::min();

    //       Eigen::Vector3d xyz_world;
    //       const Eigen::MatrixXd &pts_world = pts_worlds.at(cam_index); //[cam_index];
    //       for(size_t i = 0; i < (size_t)pts_world.cols(); ++i)
    //       {
    //         xyz_world = pts_world.col(i);

    //         svo_msgs::Feature feature;
    //         feature.x = xyz_world(0);
    //         feature.y = xyz_world(1);
    //         feature.z = xyz_world(2);
    //         msg.features.push_back(feature);

    //         Eigen::Vector3d pos_in_frame = frame_pose.inverse() * xyz_world;
    //         min_z = std::min(pos_in_frame[2], min_z);
    //         max_z = std::max(pos_in_frame[2], max_z);
    //       }
    //       msg.min_depth = (float)min_z;
    //       msg.max_depth = (float)max_z;

    //       // publish cam in world frame
    //       Transformation T_world_from_cam(frame_pose);
    //       const Eigen::Quaterniond& q =
    //           T_world_from_cam.GetRotation().ToImplementation();
    //       const Eigen::Vector3d& p = T_world_from_cam.GetPosition();

    //       msg.pose.position.x = p[0];
    //       msg.pose.position.y = p[1];
    //       msg.pose.position.z = p[2];
    //       msg.pose.orientation.w = q.w();
    //       msg.pose.orientation.x = q.x();
    //       msg.pose.orientation.y = q.y();
    //       msg.pose.orientation.z = q.z();
    //       pub_dense_.at(cam_index).publish(msg);
    //     }
    //   }
    // }

    // void Visualizer::visualizeCoordinateFrames(const Transformation& T_world_cam)
    // {
    //   if (pub_markers_.getNumSubscribers() == 0)
    //     return;

    //   // camera frame
    //   output_helper::publishFrameMarker(
    //       pub_markers_, T_world_cam.GetRotationMatrix(), T_world_cam.GetPosition(),
    //       "cam", ros::Time::now(), 0, 0, 0.2);

    //   // origin frame
    //   output_helper::publishFrameMarker(pub_markers_, Eigen::Matrix3d::Identity(),
    //                                         Eigen::Vector3d::Zero(), kWorldFrame,
    //                                         ros::Time::now(), 0, 0, 0.2);
    // }

    /*
// #ifdef SVO_LOOP_CLOSING
void Visualizer::publishLoopClosureInfo(const LoopVizInfoVec& loop_viz_info,
                                        const std::string& ns,
                                        const Eigen::Vector3f& color,
                                        const double scale)
{
  publishLineList(pub_loop_closure_, loop_viz_info, ns, color, scale);
}

bool Visualizer::publishPoseGraph(const std::vector<KeyFramePtr>& kf_list,
                                  const bool redo_pointcloud,
                                  const size_t ignored_past_frames)
{
  bool pc_recalculated = false;
  if (pub_pose_graph_.getNumSubscribers() > 0)
  {
    PointCloud pose_graph_pc;
    pcl_conversions::toPCL(ros::Time::now(), pose_graph_pc.header.stamp);
    pose_graph_pc.header.frame_id = kWorldFrame;
    pose_graph_pc.clear();
    pose_graph_pc.reserve(kf_list.size());

    // visualize all the keyframes as spheres
    for (size_t i = 0; i < kf_list.size(); i++)
    {
      const KeyFramePtr kf = kf_list[i];
      PointType pt;
      pt.x = kf->T_w_c_.GetPosition()(0);
      pt.y = kf->T_w_c_.GetPosition()(1);
      pt.z = kf->T_w_c_.GetPosition()(2);
      pt.intensity = 60;
      pose_graph_pc.push_back(pt);
    }
    pub_pose_graph_.publish(pose_graph_pc);
  }

  if (pub_pose_graph_map_.getNumSubscribers() > 0 &&
      kf_list.size() > ignored_past_frames)
  {
    if (redo_pointcloud)
    {
      // TODO: we do not need to redo it every time
      pose_graph_map_.clear();
      pcl_conversions::toPCL(ros::Time::now(), pose_graph_map_.header.stamp);
      pose_graph_map_.reserve(kf_list.size() * 60);
      for (size_t i = 0; i < kf_list.size() - ignored_past_frames; i++)
      {
        const KeyFramePtr& kf = kf_list[i];
        PointType pt;
        pt.intensity = 60;
        std::vector<cv::Point3f> pw_vec;
        kf->getLandmarksInWorld(pw_vec);
        for (const auto& lm : pw_vec)
        {
          pt.x = lm.x;
          pt.y = lm.y;
          pt.z = lm.z;
          pose_graph_map_.push_back(pt);
        }
      }
      pc_recalculated = true;
    }
    else
    {
      pcl_conversions::toPCL(ros::Time::now(), pose_graph_map_.header.stamp);
      const KeyFramePtr kf = kf_list[kf_list.size() - ignored_past_frames];
      PointType pt;
      pt.intensity = 60;
      std::vector<cv::Point3f> pw_vec;
      kf->getLandmarksInWorld(pw_vec);
      for (const auto& lm : pw_vec)
      {
        pt.x = lm.x;
        pt.y = lm.y;
        pt.z = lm.z;
        pose_graph_map_.push_back(pt);
      }
    }
    pub_pose_graph_map_.publish(pose_graph_map_);
  }
  return pc_recalculated;
}
// #endif
*/

    // void Visualizer::writeCaptionStr(cv::Mat img_rgb)
    // {
    //   if (viz_caption_str_)
    //   {
    //     cv::putText(img_rgb, img_caption_, cv::Point(20, 20),
    //                 cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,250),
    //                 1, cv::LINE_AA);
    //   }
    // }

    // void Visualizer::DrawFeatures(const std::vector<cv::Mat> &img_pyr,
    //                   const Eigen::MatrixXd &feat_px,
    //                   const Eigen::MatrixXd &feat_grad,
    //                   const Eigen::MatrixXi &feat_status,
    //                   const bool only_matched_features,
    //                   const size_t level,
    //                   cv::Mat* img_rgb)
    // {
    //   CHECK_NOTNULL(img_rgb);
    //   CHECK_GT(img_pyr.size(), level);

    //   const int scale = (1<<level);
    //   const int size = (level == 1) ? 2 : 0;
    //   *img_rgb = cv::Mat(img_pyr[level].size(), CV_8UC3);
    //   cv::cvtColor(img_pyr[level], *img_rgb, cv::COLOR_GRAY2RGB);

    //   if(level == 0)
    //   {
    //     for(size_t i = 0; i < (size_t)feat_px.cols(); ++i)
    //     {
    //       const auto& px = feat_px.col(i);
    //       const auto &g = feat_grad.col(i);
    //       const auto &status = feat_status.col(i);

    //       int type = status[0];
    //       bool landmark_valid = (bool)status[1];
    //       bool seed_refkf_valid = (bool)status[2];

    //       // if(frame.landmark_vec_[i] == nullptr
    //       //    && frame.seed_ref_vec_[i].keyframe == nullptr
    //       //    && only_matched_features)
    //       //   continue;

    //       if(!landmark_valid && !seed_refkf_valid && only_matched_features)
    //         continue;

    //       // const auto& g = frame.grad_vec_.col(i);
    //       switch (type)
    //       {
    //         case 6: // FeatureType::kEdgelet:
    //           cv::line(*img_rgb, cv::Point2f(px(0) + 3 * g(1), px(1) - 3 * g(0)),
    //                    cv::Point2f(px(0) - 3 * g(1), px(1) + 3 * g(0)),
    //                    cv::Scalar(255, 0, 255), 2);
    //           break;
    //         case 7: // FeatureType::kCorner:
    //           cv::rectangle(*img_rgb, cv::Point2f(px(0) - 2, px(1) - 2),
    //                         cv::Point2f(px(0) + 2, px(1) + 2),
    //                         cv::Scalar(0, 255, 0), -1);
    //           break;
    //         case 8: // FeatureType::kMapPoint:
    //           cv::rectangle(*img_rgb, cv::Point2f(px(0) - 2, px(1) - 2),
    //                         cv::Point2f(px(0) + 2, px(1) + 2),
    //                         cv::Scalar(255, 0, 0), -1);
    //           break;
    //         case 9: // FeatureType::kFixedLandmark:
    //           cv::rectangle(*img_rgb, cv::Point2f(px(0) - 3, px(1) - 3),
    //                         cv::Point2f(px(0) + 3, px(1) + 3),
    //                         cv::Scalar(101, 236, 255), -1);
    //           break;
    //         case 0: // FeatureType::kEdgeletSeed:
    //         case 3: //FeatureType::kEdgeletSeedConverged:
    //           cv::line(*img_rgb, cv::Point2f(px(0) + 3 * g(1), px(1) - 3 * g(0)),
    //                    cv::Point2f(px(0) - 3 * g(1), px(1) + 3 * g(0)),
    //                    cv::Scalar(0, 0, 255), 2);
    //           break;
    //         case 1: // FeatureType::kCornerSeed:
    //         case 4: // FeatureType::kCornerSeedConverged:
    //           cv::circle(*img_rgb, cv::Point2f(px(0), px(1)),
    //                      5, cv::Scalar(0, 255, 0), 1);
    //           break;
    //         case 2: // FeatureType::kMapPointSeed:
    //         case 5: // FeatureType::kMapPointSeedConverged:
    //           cv::circle(*img_rgb, cv::Point2f(px(0), px(1)),
    //                      5, cv::Scalar(255, 0, 0), 1);
    //           break;
    //         default:
    //           cv::circle(*img_rgb, cv::Point2f(px(0), px(1)),
    //                      5, cv::Scalar(0, 0, 255), -1);
    //           break;
    //       }
    //     }
    //   }
    //   else
    //   {
    //     for(size_t i = 0; i < (size_t)feat_px.cols(); ++i)
    //     {
    //       const auto& px = feat_px.col(i);
    //       auto &status = feat_status.col(i);
    //       bool landmark_valid = (bool)status[1];
    //       bool is_seed = (bool)status[2];

    //       // if(frame.IsValidLandmark(i))
    //       if(landmark_valid)
    //       {
    //         cv::rectangle(*img_rgb,
    //                       cv::Point2f(px(0)/scale-size, px(1)/scale-size),
    //                       cv::Point2f(px(0)/scale+size, px(1)/scale+size),
    //                       cv::Scalar(0,255,0), -1);
    //       }
    //       // else if(isCornerEdgeletSeed(frame.type_vec_[i]))
    //       else if(is_seed)
    //       {
    //         cv::rectangle(*img_rgb,
    //                       cv::Point2f(px(0)/scale-size, px(1)/scale-size),
    //                       cv::Point2f(px(0)/scale+size, px(1)/scale+size),
    //                       cv::Scalar(255,0,255), -1);
    //       }
    //     }
    //   }
    // }

} // end namespace mivins
