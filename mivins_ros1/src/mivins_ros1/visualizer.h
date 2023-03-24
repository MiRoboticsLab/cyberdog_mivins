// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).

#pragma once

#include <vector>
#include <utility> // std::pair
#include <iostream>
#include <unordered_map>

#include <boost/shared_ptr.hpp>

// ros
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
//#include <kindr/minimal/quat_transformation.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

// #ifdef SVO_LOOP_CLOSING
/*
#include <mivins/online_loopclosing/keyframe.h>
#include <mivins/online_loopclosing/loop_closing.h>
*/
// #endif

namespace mivins
{
    // using Transformation = kindr::minimal::QuatTransformation;
    // using Quaternion = kindr::minimal::RotationQuaternion;

    // forward declarations
    //class ChannelFrameBase;

    /// Publish visualisation messages to ROS.
    class Visualizer
    {
    public:
        typedef std::shared_ptr<Visualizer> Ptr;
        typedef pcl::PointXYZI PointType;
        typedef pcl::PointCloud<PointType> PointCloud;

        static std::string kWorldFrame;

        static constexpr double seed_marker_scale_ = 0.03;
        static constexpr double seed_uncertainty_marker_scale_ = 0.03;
        static constexpr double trajectory_marker_scale_ = 0.03;
        static constexpr double point_marker_scale_ = 0.05;

        ros::NodeHandle pnh_;
        size_t trace_id_ = 0;
        std::string trace_dir_;
        size_t img_pub_level_;
        size_t img_pub_nth_;
        size_t dense_pub_nth_;
        bool viz_caption_str_;

        ros::Publisher pub_frames_;
        ros::Publisher pub_points_;
        ros::Publisher pub_imu_pose_;
        ros::Publisher pub_info_;
        ros::Publisher pub_markers_;
        ros::Publisher pub_pc_;
        ros::Publisher pub_odometry, pub_odometryleft, pub_odometryright;
        PointCloud::Ptr pc_;
        std::vector<ros::Publisher> pub_cam_poses_;
        std::vector<ros::Publisher> pub_dense_;
        std::vector<image_transport::Publisher> pub_images_;

        tf::TransformBroadcaster br_;
        bool publish_world_in_cam_frame_;
        bool publish_map_every_frame_;
        ros::Duration publish_points_display_time_;
        bool publish_seeds_;
        bool publish_seeds_uncertainty_;
        bool publish_active_keyframes_;
        bool trace_pointcloud_;
        double vis_scale_;
        std::ofstream ofs_pointcloud_;

        // #ifdef SVO_LOOP_CLOSING
        PointCloud pose_graph_map_;
        ros::Publisher pub_loop_closure_;
        ros::Publisher pub_pose_graph_;
        ros::Publisher pub_pose_graph_map_;
        // #endif

        ros::Publisher pub_visible_fixed_landmarks_;

        std::string img_caption_;

        Visualizer(const ros::NodeHandle &nh_private,
                   const std::string config_file,
                   const std::string trace_dir,
                   const size_t n_cameras);

        ~Visualizer() = default;

        // void publishSvoInfo(const int64_t timestamp_nanoseconds,
        //               const int stage, const int track_quality,
        //               const size_t last_obs_num, const double last_process_time);

        // void publishImages(const int cam_idx,
        //                   const std::vector<cv::Mat>& img_pyr,
        //                   const int64_t timestamp_nanoseconds);

        // void publishImagesWithFeatures(int cam_idx,
        //                               const int64_t timestamp,
        //                               const std::vector<cv::Mat> &img_pyr,
        //                               const Eigen::MatrixXd &feat_px,
        //                               const Eigen::MatrixXd &feat_grad,
        //                               const Eigen::MatrixXi &feat_status,
        //                               const bool draw_boundary);

        // void publishImuPose(const Transformation& T_world_imu,
        //                     const Eigen::Matrix<double, 6, 6> Covariance,
        //                     const int64_t timestamp_nanoseconds);

        void publishCameraPoses(std::vector<std::vector<double>> cam_poses,
                                const int64_t timestamp_nanoseconds);

        // void publishBundleFeatureTracks(const FrameBundlePtr frames_ref,
        //                                 const FrameBundlePtr frames_cur,
        //                                 int64_t timestamp);
        void publishOdometry(std::vector<std::vector<double>> cam_poses,
                             const int64_t timestamp_nanoseconds);
        // void publishFeatureTracks(
        //     const Eigen::MatrixXd& px_ref, const Eigen::MatrixXd& px_cur,
        //     const std::vector<std::pair<size_t, size_t>>& matches_ref_cur,
        //     const std::vector<cv::Mat>& img_pyr, const int& level, const uint64_t timestamp,
        //     const size_t frame_index);

        // void visualizeHexacopter(const Transformation& T_frame_world,
        //                          const uint64_t timestamp);

        // void visualizeQuadrocopter(const Transformation& T_frame_world,
        //                            const uint64_t timestamp);

        // void visualizeMarkers(const std::vector<Transformation> &last_frame_poses,
        //                       const std::vector<Transformation> &active_kf_poses,
        //                       std::unordered_map<int, Transformation> &close_kf_poses,
        //                       const std::vector<Eigen::Vector4d> &map_region_pts,
        //                       const std::vector<Eigen::Vector3d> map_seeds,
        //                       const std::vector<Eigen::Vector3d> map_seeds_uncertainty,
        //                       const std::unordered_map<int, Eigen::Vector4d> &close_kf_all_pts,
        //                       const int64_t timestamp, const bool is_kf);

        // void publishTrajectoryPoint(const Eigen::Vector3d& pos_in_vision,
        //                             const uint64_t timestamp, const int id);

        // void publishSeeds(std::vector<Eigen::Vector3d> map_seeds);

        // void publishVelocity(const Eigen::Vector3d& velocity_imu,
        //                      const uint64_t timestamp);

        // void publishMapRegion(const std::vector<Eigen::Vector4d> &map_regions,
        //                       const std::unordered_map<int, Transformation> &close_kf_poses,
        //                       const std::unordered_map<int, Eigen::Vector4d> &close_kf_all_pts);

        // void publishKeyframeWithPoints(const std::unordered_map<int, Transformation> &cam_poses,
        //                                const std::unordered_map<int, Eigen::Vector4d> &id_pts,
        //                                const double marker_scale = 0.05);

        // void publishActiveKeyframes(const std::vector<Transformation>& active_kf_poses);

        // void exportToDense(const uint64_t ts,
        //     const std::vector<int> frame_id,
        //     const std::vector<cv::Mat> &imgs,
        //     const std::vector<Transformation> &frame_poses,
        //     const std::vector<Eigen::MatrixXd> &pts_worlds);

        // void publishSeedsUncertainty(const std::vector<Eigen::Vector3d> &map_seeds_uncertainty);

        // void visualizeCoordinateFrames(const Transformation& T_world_cam);

        /*
  void publishLoopClosureInfo(
      const LoopVizInfoVec& loop_viz_info_vec,
      const std::string& ns, const Eigen::Vector3f& color,
      const double scale=1.0);

  bool publishPoseGraph(const std::vector<KeyFramePtr>& kf_list,
                        const bool redo_pointcloud,
                        const size_t ignored_past_frames);

  */

        //   void writeCaptionStr(cv::Mat img);

        //   void DrawFeatures(const std::vector<cv::Mat> &img_pyr,
        //                 const Eigen::MatrixXd &feat_px,
        //                 const Eigen::MatrixXd &feat_grad,
        //                 const Eigen::MatrixXi &feat_status,
        //                 const bool only_matched_features,
        //                 const size_t level,
        //                 cv::Mat* img_rgb);
    };

} // end namespace mivins
