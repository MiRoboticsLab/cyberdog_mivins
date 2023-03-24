#pragma once

#include <utility> // std::pair
#include <iostream>

#include <boost/shared_ptr.hpp>

// ros
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <std_msgs/msg/color_rgba.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <queue>
#include "nav2_util/lifecycle_node.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
namespace mivins
{
    typedef struct RelocResponse {
     
        int reloc_id;
        bool is_verified;
        float confidence;
        int64_t time;
        int status;
        double x;
        double y;
        double z;
        double rx;
        double ry;
        double rz;
        double rw;
    }RelocResponse;

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

        //rclcpp::Node *pnh_;
        rclcpp::Node::SharedPtr pnh_;
        //nav2_util::LifecycleNode *pnh_;
        size_t trace_id_ = 0;
        std::string trace_dir_;
        size_t img_pub_level_;
        size_t img_pub_nth_;
        size_t dense_pub_nth_;
        bool viz_caption_str_;

        //rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_frames_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_points_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_imu_pose_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_markers_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_imu_odometry;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_reloc_odometry;//for test
        
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_baselink_odometry;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometryleft;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometryright;

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_pathleft;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_pathright;

        PointCloud::Ptr pc_;
        nav_msgs::msg::Path path;
        nav_msgs::msg::Path pathleft;
        nav_msgs::msg::Path pathright;
        std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pub_cam_poses_;
        std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> pub_images_;

        //tf2_ros::TransformBroadcaster br_;
        bool publish_world_in_cam_frame_;
        bool publish_map_every_frame_;
        //rclcpp::Duration publish_points_display_time_;
        bool publish_seeds_;
        bool publish_seeds_uncertainty_;
        bool publish_active_keyframes_;
        bool trace_pointcloud_;
        double vis_scale_;
        std::ofstream ofs_pointcloud_;
 
        std::string img_caption_;

        Visualizer(//rclcpp::Node *nh_private,
                   rclcpp::Node::SharedPtr nh_private,
                   //nav2_util::LifecycleNode  *nh_private,
                   const std::string config_file,
                   const std::string trace_dir,
                   const size_t num_cameras);

        ~Visualizer() = default;

        void pubOdometry(std::vector<std::vector<double>> cam_poses,
                         const int64_t timestamp_nanoseconds);
        void publishCameraPoses(std::vector<std::vector<double>> cam_poses,
                                const int64_t timestamp_nanoseconds);
        Eigen::Affine3d publishBaselinkToVodomTF(double time_stamp,
                                     std::vector<double> cam_pose);
	    void publishBaselinkToVodomTF(double time_stamp, Eigen::Affine3d tf_vodom_baselink);
        void publishVodomToMapTF(double time_stamp, Eigen::Affine3d tf_vodom2map);
        Eigen::Affine3d publishVodomToMapTF(double time_stamp,
                                            std::vector<double> cam_pose,
                                            std::queue<RelocResponse> relocResponses,
                                            std::vector<double> imu_to_calib,
                                            std::vector<double> baselink_to_imu_calib);
        void publishVodomToMapTFStatic(double time_stamp);
        void pubIMUOdometry(double time_stamp,std::vector<double> imu_pose);
        void publishBaselinkToMapOdometry(double timestamp_nanoseconds,Eigen::Affine3d baselink_to_map,const int &mode);
        void publishBaselinkToMapTest(double time_stamp,std::vector<double> base_to_map);
        void visualizerReset();
        // void init();

        Eigen::Affine3d vodom_to_map_;
        int64_t time_vodom_to_map_;
        bool init_reloc_;
        std::ofstream save_baselink_map_;//for reloc test
        double last_new_reloc_time_;
       
    };

} // end namespace mivins
