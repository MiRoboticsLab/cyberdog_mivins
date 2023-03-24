#include <visualizer.h>

#include <deque>
#include <algorithm>
#include <iostream>
#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/time.h>
#include <tf2_ros/async_buffer_interface.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
//#include <tf/tf.h>
//#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <yaml-cpp/yaml.h>
#pragma diagnostic pop

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

    Visualizer::Visualizer(rclcpp::Node::SharedPtr nh_private,//rclcpp::Node *nh_private,//nav2_util::LifecycleNode *nh_private,//
                           const std::string config_file,
                           const std::string trace_dir,
                           const size_t num_cameras)
        : pnh_(nh_private), trace_dir_(trace_dir)
          //, publish_points_display_time_(
          //      getParam<double>(config, "publish_point_display_time", 0))
          ,
          pc_(new PointCloud)
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
        //publish_points_display_time_ = rclcpp::Duration(
        //      getParam<double>(config, "publish_point_display_time", 0));
        publish_seeds_ = getParam<bool>(config, "publish_seeds", true);
        publish_seeds_uncertainty_ =
            getParam<bool>(config, "publish_seeds_uncertainty", false);
        publish_active_keyframes_ = getParam<bool>(config, "publish_active_kfs", false);
        trace_pointcloud_ = getParam<bool>(config, "trace_pointcloud", false);
        vis_scale_ = getParam<double>(config, "publish_marker_scale", 1.2);

        pub_markers_ = pnh_->create_publisher<visualization_msgs::msg::Marker>("markers", 1000);
        pub_pc_ = pnh_->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 1000);
        pub_odometry = pnh_->create_publisher<nav_msgs::msg::Odometry>("mivins/odometry", 1000);
        pub_imu_odometry = pnh_->create_publisher<nav_msgs::msg::Odometry>("mivins/imuodom_slam", 1000);
        pub_baselink_odometry = pnh_->create_publisher<nav_msgs::msg::Odometry>("odom_slam", 1000);//imu to map
        pub_path = pnh_->create_publisher<nav_msgs::msg::Path>("mivins/path", 1000);
        pub_odometryleft = pnh_->create_publisher<nav_msgs::msg::Odometry>("mivins/odometryleft", 1000);
        pub_pathleft = pnh_->create_publisher<nav_msgs::msg::Path>("mivins/pathleft", 1000);
        pub_odometryright = pnh_->create_publisher<nav_msgs::msg::Odometry>("mivins/odometryright", 1000);
        pub_pathright = pnh_->create_publisher<nav_msgs::msg::Path>("mivins/pathright", 1000);
        pub_reloc_odometry = pnh_->create_publisher<nav_msgs::msg::Odometry>("mivins/reloc_odom", 1000);;//for test
        //pub_dense_.resize(num_cameras);
        pub_images_.resize(num_cameras);
        pub_cam_poses_.resize(num_cameras);
        //image_transport::ImageTransport it(pnh_);
        for (size_t i = 0; i < num_cameras; ++i)
        {
            pub_images_.at(i) = pnh_->create_publisher<sensor_msgs::msg::Image>("mivins/image" + std::to_string(i), 1000);
            pub_cam_poses_.at(i) = pnh_->create_publisher<geometry_msgs::msg::PoseStamped>(
                "pose_cam/cam" + std::to_string(i), 10);
        }
        init_reloc_ = false;
        vodom_to_map_= Eigen::Affine3d::Identity();
        save_baselink_map_.open("/SDCARD/mivins/baselinkTomap_trajectory.txt");
        last_new_reloc_time_ = 0;
        
    }
    void Visualizer::visualizerReset() {
        pub_markers_.reset();
        pub_pc_ .reset();
        pub_odometry.reset();
        pub_imu_odometry.reset();
        pub_baselink_odometry.reset();//imu to map
        pub_path.reset();
        pub_odometryleft.reset();
        pub_pathleft.reset();
        pub_odometryright.reset();
        pub_pathright.reset();
        pub_reloc_odometry.reset();//for test
    }
    // void Visualizer::init() {
    //     init_reloc_ = false;
    //     vodom_to_map_= Eigen::Affine3d::Identity();
    //     save_baselink_map_.open("/SDCARD/mivins/baselinkTomap_trajectory.txt");
    //     last_new_reloc_time_ = 0;
    // }

    void Visualizer::publishCameraPoses(std::vector<std::vector<double>> cam_poses,
                                        const int64_t timestamp_nanoseconds)
    {
        INFO_STREAM("[MIVINS][PUBLISHCAMERAPOSES]publishCameraPoses " << cam_poses[0][0] );
        if (cam_poses.empty())
            return;
        for (size_t i = 0; i < cam_poses.size(); ++i)
        {
            geometry_msgs::msg::PoseStamped::SharedPtr msg_pose(new geometry_msgs::msg::PoseStamped);
            //msg_pose->header.seq = trace_id_;
            msg_pose->header.stamp = tf2_ros::toMsg(tf2::timeFromSec(timestamp_nanoseconds * 1e-9));
            msg_pose->header.frame_id = "cam" + std::to_string(i);
            msg_pose->pose.position.x = cam_poses[i][0];
            msg_pose->pose.position.y = cam_poses[i][1];
            msg_pose->pose.position.z = cam_poses[i][2];
            msg_pose->pose.orientation.x = cam_poses[i][3];
            msg_pose->pose.orientation.y = cam_poses[i][4];
            msg_pose->pose.orientation.z = cam_poses[i][5];
            msg_pose->pose.orientation.w = cam_poses[i][6];
            pub_cam_poses_.at(i)->publish(*msg_pose);
        }
    }
    Eigen::Affine3d Visualizer::publishVodomToMapTF(double time_stamp,
                                         std::vector<double> cam_pose,
                                         std::queue<RelocResponse> relocResponses,
                                         std::vector<double> imu_to_calib,
                                         std::vector<double> baselink_to_imu_calib)
    {
        if(relocResponses.size() < 1 || cam_pose.empty()) {
            INFO_STREAM("publishVodomToMapTF empty!");
        } else {
            //缓存，不改变queque
            double init_reloc_score_threshold = 0.5;
            std::vector<RelocResponse> vreloc;
            while(relocResponses.size() > 0) {
                vreloc.push_back(relocResponses.front());
                relocResponses.pop();
            }

            Eigen::Quaterniond q_baselink_to_imu(baselink_to_imu_calib[6],baselink_to_imu_calib[3],baselink_to_imu_calib[4],baselink_to_imu_calib[5]);
            Eigen::Translation3d t_baselink_to_imu(baselink_to_imu_calib[0],baselink_to_imu_calib[1],baselink_to_imu_calib[2]);

            Eigen::Quaterniond q_imu_to_cam(imu_to_calib[6],imu_to_calib[3],imu_to_calib[4],imu_to_calib[5]);
            Eigen::Translation3d t_imu_to_cam(imu_to_calib[0],imu_to_calib[1],imu_to_calib[2]);//current frame
//修改找不到图片跳出的机制
            if(!init_reloc_) {
                INFO_STREAM("[MIVINS][PUBLISHVODOMTOMAPTF]reloc init in ...");
                RelocResponse reloc_response = vreloc[0];//当前只用   
                // for(int posen = path.poses.size() - 1 ; posen >=0  ;posen--) 
                int posen = 0;
                {
                    INFO_STREAM("[MIVINS][PUBLISHVODOMTOMAPTF]reloc timemax : " <<std::to_string(rclcpp::Time(path.poses[posen].header.stamp).nanoseconds()/1000000000.0)<<
                                "reloc timemin : " <<std::to_string(rclcpp::Time(path.poses[0].header.stamp).nanoseconds()/1000000000.0) );
                    INFO_STREAM("[MIVINS][PUBLISHVODOMTOMAPTF]reloc path.poses[posen].header.stamp : "<<std::to_string(rclcpp::Time(path.poses[posen].header.stamp).nanoseconds())<<
                               "response.time : "<<std::to_string(rclcpp::Time(reloc_response.time).nanoseconds())   );
                    double odom_time = rclcpp::Time(path.poses[posen].header.stamp).nanoseconds()/1000000000.0;
                    double reloc_time = rclcpp::Time(reloc_response.time).nanoseconds()/1000000000.0;
                    INFO_STREAM("[MIVINS][PUBLISHVODOMTOMAPTF]odom_time: "<< std::to_string(odom_time) << "  reloc_time: "<< std::to_string(reloc_time)  );
                    // if(abs( odom_time- reloc_time) < 0.05)  //当前因为重定位第一帧找的时间问题，只能静止重定位第一帧！
                    {//取最新的reloc计算
                        INFO_STREAM("[MIVINS][PUBLISHVODOMTOMAPTF]reloc getin posetime: "<<  std::to_string(rclcpp::Time(path.poses[posen].header.stamp).nanoseconds()) << "   responseTime: " << 
                                    std::to_string(rclcpp::Time(reloc_response.time).nanoseconds()) );
                        // if(reloc_response.is_verified || reloc_response.reloc_id == 0) 
                        {
                            INFO_STREAM("[MIVINS][PUBLISHVODOMTOMAPTF]reloc getin verified:"<< reloc_response.is_verified<< " confidence: "<<  reloc_response.confidence
                                << " status: "<< reloc_response.status);
                            if(reloc_response.confidence >= init_reloc_score_threshold && reloc_response.status == 1) {
                                
                                Eigen::Quaterniond q_reloc(reloc_response.rw,
                                                        reloc_response.rx,
                                                        reloc_response.ry,
                                                        reloc_response.rz);
                                Eigen::Translation3d t_reloc(reloc_response.x,
                                                            reloc_response.y,
                                                            reloc_response.z);//reloc value    T_map_imu
                                Eigen::Quaterniond q_vodom(path.poses[posen].pose.orientation.w,
                                                        path.poses[posen].pose.orientation.x,
                                                        path.poses[posen].pose.orientation.y,
                                                        path.poses[posen].pose.orientation.z);
                                Eigen::Translation3d t_vodom(path.poses[posen].pose.position.x,
                                                            path.poses[posen].pose.position.y,
                                                            path.poses[posen].pose.position.z);// vodom value  T_vodom_camera
                                Eigen::Quaterniond qc_vodom(cam_pose[6],cam_pose[3],cam_pose[4],cam_pose[5]);
                                Eigen::Translation3d tc_vodom(cam_pose[0],cam_pose[1],cam_pose[2]);//current frame
 
                                vodom_to_map_ = (t_reloc*q_reloc)*(t_baselink_to_imu* q_baselink_to_imu).inverse()*(t_imu_to_cam*q_imu_to_cam).inverse()*(t_vodom*q_vodom).inverse();//*(tc_vodom*qc_vodom)).inverse()
                                //vodom_to_map_ = (t_reloc*q_reloc)*(t_rsleft_to_baselink*q_rsleft_to_baselink)*(t_vodom*q_vodom).inverse();
                                last_new_reloc_time_ = vreloc[vreloc.size() - 1].time/1000000000.0;
                                INFO_STREAM("[MIVINS][PUBLISHVODOMTOMAPTF]reloc getin time and the vodom_to_map_ : " << vodom_to_map_.matrix());
                                INFO_STREAM("[MIVINS][PUBLISHVODOMTOMAPTF]reloc success get reloc" );
                                init_reloc_ = true;//成功reloc
                                time_vodom_to_map_ = rclcpp::Time(path.poses[posen].header.stamp).nanoseconds() ;//成功reloc
                                // break;  //for循环时打开
                            } else {
                                INFO_STREAM("[MIVINS][PUBLISHVODOMTOMAPTF]reloc confidence or status failed!" << " verified: "<< reloc_response.is_verified << "  reloc_id:"<<reloc_response.reloc_id   );
                            }
                        } 
                        // else {
                        //     INFO_STREAM("reloc verified false!" );
                        // }
                    } //for 循环是打开ia
                    // else if(rclcpp::Time(reloc_response.time).nanoseconds() - rclcpp::Time(path.poses[posen].header.stamp).nanoseconds() > 0.1 ) {
                    //     INFO_STREAM("reloc away synchronization!!!" );
                    //     // break; //for循环时打开
                    // }
                }

            } else {
                INFO_STREAM("[MIVINS][PUBLISHVODOMTOMAPTF]reloc find in ..." );
                // double distance_min = 0.5;//设定阈值reloc判断是否有效
                if( vreloc.size() < 3) {
                    INFO_STREAM("[MIVINS][PUBLISHVODOMTOMAPTF]reloc : vreloc.size() < 3");
                } else if (vreloc.size() > 3) {
                    INFO_STREAM("[MIVINS][PUBLISHVODOMTOMAPTF]reloc : vreloc.size() > 3");
                } else {
                    INFO_STREAM("[MIVINS][PUBLISHVODOMTOMAPTF]reloc : vreloc.size() = 3  yes!");//确定个数是有3个：这个3是对应visualizer  int reloc_num_find_abnormal = 3保持一致
                }
                std::vector<int64_t> n_find_index;
                for(int rn = 0;rn < vreloc.size();rn++ ) {

                    RelocResponse reloc_response = vreloc[rn];//当前只用
                    
                    double reloc_time = rclcpp::Time(reloc_response.time).nanoseconds()/1000000000.0;//各个重定位时间
                    
                    for(int posen = path.poses.size() - 1 ; posen >=0  ;posen--) {
                        double odom_time = rclcpp::Time(path.poses[posen].header.stamp).nanoseconds()/1000000000.0;
                        if(abs( odom_time- reloc_time) < 0.1)
                        {//取最新的reloc计算
                            // std::endl;
                            n_find_index.push_back(posen);
                            break;
                        }
                    }
                }
                double last_reloc_time = time_vodom_to_map_/1000000000.0;//上一次重定位pose对应的time
                double reloc_new_time = rclcpp::Time(vreloc[vreloc.size() - 1].time).nanoseconds()/1000000000.0;
                double reloc_dt = last_reloc_time - reloc_new_time;//判断是否新的reloctime距离上次good信息太近(这个判断可以去掉)
                double reloc_new_time_dt = last_new_reloc_time_ - reloc_new_time;//判断是否有新的reloctime加入queue,避免过多重复计算
                if(n_find_index.size() != vreloc.size() || abs(reloc_dt) < 0.5) {
                    INFO_STREAM("[MIVINS][PUBLISHVODOMTOMAPTF]reloc : n_find_index.size() "<< n_find_index.size()<<"!= vreloc.size()"<< vreloc.size());//这里如果有reloc信息没有找到mivins对应的pose则不进行更新vodom_map的tf
                    // break;
                    INFO_STREAM("[MIVINS][PUBLISHVODOMTOMAPTF]reloc reloc_dt: " << reloc_dt);
                } else if(abs(reloc_new_time_dt) > 0.05){

                    double distance_min = 1000.0;
                    int reloc_n = -1;
                    Eigen::Affine3d vodom_to_map_good;
                    for(int rn = 0;rn < vreloc.size();rn++ ) {
                        RelocResponse reloc_response = vreloc[rn];
                        // if(reloc_response.confidence >= 0.8 && reloc_response.status == 1) {
                        //     INFO_STREAM("reloc pub TF" );
                        //     continue;
                        // }
                        Eigen::Quaterniond q_reloc(reloc_response.rw,
                                                reloc_response.rx,
                                                reloc_response.ry,
                                                reloc_response.rz);
                        Eigen::Translation3d t_reloc(reloc_response.x,
                                                    reloc_response.y,
                                                    reloc_response.z);//reloc value    T_map_imu
                        Eigen::Quaterniond q_vodom(path.poses[n_find_index[rn]].pose.orientation.w,
                                                path.poses[n_find_index[rn]].pose.orientation.x,
                                                path.poses[n_find_index[rn]].pose.orientation.y,
                                                path.poses[n_find_index[rn]].pose.orientation.z);
                        Eigen::Translation3d t_vodom(path.poses[n_find_index[rn]].pose.position.x,
                                                    path.poses[n_find_index[rn]].pose.position.y,
                                                    path.poses[n_find_index[rn]].pose.position.z);// vodom value  T_vodom_camera
 
                        // Eigen::Affine3d vodom_to_map_temp = (t_reloc*q_reloc)*    *(t_imu_to_cam*q_imu_to_cam).inverse()*(t_vodom*q_vodom).inverse(); 
                        //Eigen::Affine3d vodom_to_map_temp = (t_reloc*q_reloc)*(t_rsleft_to_baselink*q_rsleft_to_baselink)*(t_vodom*q_vodom).inverse();
                        Eigen::Affine3d vodom_to_map_temp = (t_reloc*q_reloc)*(t_baselink_to_imu* q_baselink_to_imu).inverse()*
                                                            (t_imu_to_cam*q_imu_to_cam).inverse()*(t_vodom*q_vodom).inverse();//*(tc_vodom*qc_vodom)).inverse()
                                
                        double distance_accinitreloc_newrelocvalue = 0;
                        for(int pn = 0;pn < vreloc.size() ;pn++) {
                            if(pn == rn) {
                                continue;
                            }
                            Eigen::Quaterniond q_vodom(path.poses[n_find_index[pn]].pose.orientation.w,
                                                path.poses[n_find_index[pn]].pose.orientation.x,
                                                path.poses[n_find_index[pn]].pose.orientation.y,
                                                path.poses[n_find_index[pn]].pose.orientation.z);
                            Eigen::Translation3d t_vodom(path.poses[n_find_index[pn]].pose.position.x,
                                                        path.poses[n_find_index[pn]].pose.position.y,
                                                        path.poses[n_find_index[pn]].pose.position.z);// vodom value  T_vodom_camera
                            //使用Init对应的vodom_to_map_转换到map，得到该frame使用init_reloc信息后的坐标,与 reloc value    T_map_imu信息对比看误差大小
                            Eigen::Affine3d imu_to_map_useinit = vodom_to_map_temp*(t_vodom*q_vodom)*(t_imu_to_cam*q_imu_to_cam);
                            Eigen::Matrix4d imu_to_map_useinit_matrix = imu_to_map_useinit.matrix();//前面已经计算了是非Init time对应的，即另外的reloc信息
                            double imu_to_map_useinit_x = imu_to_map_useinit_matrix(0,3);
                            double imu_to_map_useinit_y = imu_to_map_useinit_matrix(1,3);
                            double imu_to_map_useinit_z = imu_to_map_useinit_matrix(2,3);
                            RelocResponse reloc_response_compare = vreloc[pn];
                            double distance_compare = sqrt((reloc_response_compare.x - imu_to_map_useinit_x)*(reloc_response_compare.x - imu_to_map_useinit_x)
                                                          +(reloc_response_compare.y - imu_to_map_useinit_y)*(reloc_response_compare.y - imu_to_map_useinit_y)
                                                          +(reloc_response_compare.z - imu_to_map_useinit_z)*(reloc_response_compare.z - imu_to_map_useinit_z));
                            distance_accinitreloc_newrelocvalue +=distance_compare;
                            INFO_STREAM("[MIVINS][PUBLISHVODOMTOMAPTF]reloc n_time: "<< std::to_string(rclcpp::Time(reloc_response.time).nanoseconds()/1000000000.0) << "   rn:"<<rn <<
                                    "   pn : "<<pn << " distance: "<< distance_compare );

                        }
                        if(distance_min  > distance_accinitreloc_newrelocvalue) {
                            reloc_n = rn;
                            vodom_to_map_good = vodom_to_map_temp;
                            distance_min = distance_accinitreloc_newrelocvalue;
                            INFO_STREAM("[MIVINS][PUBLISHVODOMTOMAPTF]reloctest : temp_min_distance : "<< distance_min << "   select: "<< rn  );
                        } 

                    }
                    if(reloc_n!= -1) {
                        time_vodom_to_map_ = rclcpp::Time(vreloc[reloc_n].time).nanoseconds();//成功reloc
                        vodom_to_map_ = vodom_to_map_good;
                        last_new_reloc_time_ = vreloc[vreloc.size() - 1].time/1000000000.0;
                        INFO_STREAM("[MIVINS][PUBLISHVODOMTOMAPTF]reloc : min_distance : "<< distance_min << "   select: "<< reloc_n  );
                        INFO_STREAM("[MIVINS][PUBLISHVODOMTOMAPTF]reloc : x  : " << vreloc[reloc_n].x << "reloc : y  : " << vreloc[reloc_n].y<< "reloc : z  : " << vreloc[reloc_n].z   );
                        INFO_STREAM("[MIVINS][PUBLISHVODOMTOMAPTF]reloc : time : "<< std::to_string(rclcpp::Time(vreloc[reloc_n].time).nanoseconds()/1000000000.0) );
                        INFO_STREAM("[MIVINS][PUBLISHVODOMTOMAPTF]reloc verified: "<< (int)vreloc[reloc_n].is_verified << " status: "<< vreloc[reloc_n].status << "  reloc_id: "<<  vreloc[reloc_n].reloc_id  );
                        INFO_STREAM("[MIVINS][PUBLISHVODOMTOMAPTF]reloc confidence_threshold: "<< vreloc[reloc_n].confidence );
                    }
                }

            }
        }
        static tf2_ros::TransformBroadcaster vodommapbr(pnh_);
        geometry_msgs::msg::TransformStamped transformStamped = tf2::eigenToTransform(vodom_to_map_);
        transformStamped.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(time_stamp));// rclcpp::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "vodom";
        vodommapbr.sendTransform(transformStamped);
        INFO_STREAM("[MIVINS][PUBLISHVODOMTOMAPTF]reloc vodom_to_map matrix:"<<  vodom_to_map_.matrix());
        return vodom_to_map_;
    }
    void Visualizer::publishVodomToMapTFStatic(double time_stamp)
    {
        Eigen::Affine3d vodom_to_map_static = Eigen::Affine3d::Identity();
        static tf2_ros::TransformBroadcaster vodommapbr_static(pnh_);
        geometry_msgs::msg::TransformStamped transformStamped = tf2::eigenToTransform(vodom_to_map_static);
        transformStamped.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(time_stamp));
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "vodom";
        vodommapbr_static.sendTransform(transformStamped);
        return;
    }
    void Visualizer::publishBaselinkToMapOdometry(double timestamp_nanoseconds,Eigen::Affine3d baselink_to_map,const int &mode)
    {
        Eigen::Matrix4d baselink_to_map_matrix = baselink_to_map.matrix();
        Eigen::Matrix3d baselink_to_map_matrix_3d;
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                baselink_to_map_matrix_3d(i,j) = baselink_to_map_matrix(i,j);
            }
        }
        Eigen::Quaterniond q_baselink(baselink_to_map_matrix_3d);
        Eigen::Translation3d t_baselink(baselink_to_map_matrix(0,3),baselink_to_map_matrix(1,3),baselink_to_map_matrix(2,3));
        nav_msgs::msg::Odometry odometry;
        //odometry.header = header;
        odometry.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(timestamp_nanoseconds));
        odometry.header.frame_id = "map";//
        odometry.child_frame_id = "base_link";

        odometry.pose.pose.position.x = baselink_to_map_matrix(0,3);
        odometry.pose.pose.position.y = baselink_to_map_matrix(1,3);
        odometry.pose.pose.position.z = baselink_to_map_matrix(2,3);//z轴置为0？
        if(mode == 2) {
            odometry.pose.pose.position.z = 0.2;
        }
        // odometry.pose.pose.position.z = 0;//z轴置为0

        odometry.pose.pose.orientation.x = q_baselink.x();
        odometry.pose.pose.orientation.y = q_baselink.y();
        odometry.pose.pose.orientation.z = q_baselink.z();
        odometry.pose.pose.orientation.w = q_baselink.w();
        odometry.twist.twist.linear.x = 0;
        odometry.twist.twist.linear.y = 0;
        odometry.twist.twist.linear.z = 0;

        pub_baselink_odometry->publish(odometry);
        save_baselink_map_.precision(16);
        save_baselink_map_ << std::to_string(timestamp_nanoseconds) << " "
                << odometry.pose.pose.position.x << " " << odometry.pose.pose.position.y << " " << odometry.pose.pose.position.z << " "
                << odometry.pose.pose.orientation.x << " " << odometry.pose.pose.orientation.y << " " << odometry.pose.pose.orientation.z << " " << odometry.pose.pose.orientation.w<<std::endl;
                
    }

    void Visualizer::pubIMUOdometry(double time_stamp, std::vector<double> imu_pose)
    {
        nav_msgs::msg::Odometry odometry;
        //odometry.header = header;
        odometry.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(time_stamp));
        odometry.header.frame_id = "vodom";
        odometry.child_frame_id = "imu";

        odometry.pose.pose.position.x = imu_pose[0];
        odometry.pose.pose.position.y = imu_pose[1];
        odometry.pose.pose.position.z = imu_pose[2];
        odometry.pose.pose.orientation.x = imu_pose[3];
        odometry.pose.pose.orientation.y = imu_pose[4];
        odometry.pose.pose.orientation.z = imu_pose[5];
        odometry.pose.pose.orientation.w = imu_pose[6];
        odometry.twist.twist.linear.x = 0;
        odometry.twist.twist.linear.y = 0;
        odometry.twist.twist.linear.z = 0;

        pub_imu_odometry->publish(odometry);
    }
    void Visualizer::pubOdometry(std::vector<std::vector<double>> cam_poses,
                                 const int64_t timestamp_nanoseconds)
    {
        if (cam_poses.empty())
            return;

        for (size_t i = 0; i < cam_poses.size(); ++i)
        {
            nav_msgs::msg::Odometry odometry;
            //odometry.header = header;
            odometry.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(timestamp_nanoseconds * 1e-9));
            odometry.header.frame_id = "vodom";
            // msg.header.frame_id = "world";
            // msg.header.stamp = timestamp;
            odometry.child_frame_id = "vodom";

            odometry.pose.pose.position.x = cam_poses[i][0];
            odometry.pose.pose.position.y = cam_poses[i][1];
            odometry.pose.pose.position.z = cam_poses[i][2];//cam_poses[i][2]  TODO:导航模式置为0
            odometry.pose.pose.orientation.x = cam_poses[i][3];
            odometry.pose.pose.orientation.y = cam_poses[i][4];
            odometry.pose.pose.orientation.z = cam_poses[i][5];
            odometry.pose.pose.orientation.w = cam_poses[i][6];
            odometry.twist.twist.linear.x = 0;
            odometry.twist.twist.linear.y = 0;
            odometry.twist.twist.linear.z = 0;

            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = odometry.header;
            pose_stamped.header.frame_id = "vodom";
            pose_stamped.pose = odometry.pose.pose;

            //"path"

            if (i == 0)
            {
                pub_odometry->publish(odometry);
                path.header = odometry.header;
                path.header.frame_id = "vodom";
                path.poses.push_back(pose_stamped);
                //pub_path->publish(path);
            }
        }
    }

    Eigen::Affine3d Visualizer::publishBaselinkToVodomTF(double time_stamp,std::vector<double> cam_pose)//rclcpp::Node *node,
    {
        Eigen::Quaterniond q_rsleft_to_baselink(0.4497752,-0.545621, 0.545621, -0.4497752);
        Eigen::Translation3d t_rsleft_to_baselink(0.275309,0.025,0.114282);

        static tf2_ros::TransformBroadcaster br(pnh_);
        Eigen::Quaterniond q_vodom(cam_pose[6],cam_pose[3],cam_pose[4],cam_pose[5]);
        Eigen::Translation3d t_vodom(cam_pose[0],cam_pose[1],cam_pose[2]);
        
        Eigen::Affine3d trans_odom_baselink = (t_vodom*q_vodom)*(t_rsleft_to_baselink*q_rsleft_to_baselink).inverse();// T_vodom_baselink
        geometry_msgs::msg::TransformStamped transformStamped = tf2::eigenToTransform(trans_odom_baselink);
        transformStamped.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(time_stamp));// rclcpp::Time::now();
        transformStamped.header.frame_id = "vodom";
        transformStamped.child_frame_id = "base_link";
        br.sendTransform(transformStamped);
        return trans_odom_baselink;
    }

    void Visualizer::publishBaselinkToVodomTF(double time_stamp,Eigen::Affine3d tf_vodom_baselink)//rclcpp::Node *node,
    {
        static tf2_ros::TransformBroadcaster br(pnh_);

        geometry_msgs::msg::TransformStamped transformStamped = tf2::eigenToTransform(tf_vodom_baselink);
        transformStamped.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(time_stamp));// rclcpp::Time::now();
        transformStamped.header.frame_id = "vodom";
        transformStamped.child_frame_id = "base_link";
        br.sendTransform(transformStamped);
    }

    void Visualizer::publishVodomToMapTF(double time_stamp, Eigen::Affine3d tf_vodom2map)
    {
        static tf2_ros::TransformBroadcaster br(pnh_);

        geometry_msgs::msg::TransformStamped transformStamped = tf2::eigenToTransform(tf_vodom2map);
        transformStamped.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(time_stamp));// rclcpp::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "vodom";
        br.sendTransform(transformStamped);

    }

    void Visualizer::publishBaselinkToMapTest(double time_stamp,std::vector<double> base_to_map){
        
        nav_msgs::msg::Odometry odometry;
        //odometry.header = header;
        odometry.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(time_stamp));
        odometry.header.frame_id = "map";
        odometry.child_frame_id = "base_link_reloc";

        odometry.pose.pose.position.x = base_to_map[0];
        odometry.pose.pose.position.y = base_to_map[1];
        odometry.pose.pose.position.z = base_to_map[2];
        odometry.pose.pose.orientation.x = base_to_map[3];
        odometry.pose.pose.orientation.y = base_to_map[4];
        odometry.pose.pose.orientation.z = base_to_map[5];
        odometry.pose.pose.orientation.w = base_to_map[6];
        odometry.twist.twist.linear.x = 0;
        odometry.twist.twist.linear.y = 0;
        odometry.twist.twist.linear.z = 0;

        pub_reloc_odometry->publish(odometry);

    }
} // end namespace mivins
