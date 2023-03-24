#include "svo_process.h"
#include <ros/callback_queue.h>

// #include <visualizer.h>
// #include <mivins/common/frame.h>
// #include <mivins/frontend_local_map.h>
// #include <svo/param.h>
// #include <mivins/common/camera.h>
// #include <mivins/common/conversions.h>
// #include <mivins/initialization.h>
// #include <mivins/direct/depth_optimization.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
//#include <mivins/utils/timer.h>

//#include <mivins/online_loopclosing/loop_closing.h>

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <yaml-cpp/yaml.h>
#pragma diagnostic pop

namespace mivins
{

    SvoProcess::SvoProcess(
        const ros::NodeHandle &nh,
        const ros::NodeHandle &private_nh)
        : nh_(nh), pnh_(private_nh)
    {
        pnh_.param<std::string>("calib_file", calib_file_, "");
        pnh_.param<std::string>("config_file", config_file_, "");
        FILE *fh_config = fopen(config_file_.c_str(), "r");

        if (fh_config == NULL)
        {
            ROS_INFO_STREAM("config_file path: " << config_file_);
            ROS_ERROR("config_file dosen't exist; wrong config_file path");
            ROS_BREAK();
            return;
        }
        fclose(fh_config);

        FILE *fh_calib = fopen(calib_file_.c_str(), "r");
        if (fh_calib == NULL)
        {
            ROS_INFO_STREAM("calib file path: " << calib_file_);
            ROS_ERROR("calib_file dosen't exist; wrong calib_file path");
            ROS_BREAK();
            return;
        }
        fclose(fh_calib);

        //svo_interface_ = std::make_shared<mivins::SvoInterface>(config_file_, calib_file_);
        MiVins_StartUp(config_file_, calib_file_);

        std::string trace_dir = "/home/wangcp/";
        size_t cam_size = 3; //svo_interface_->getCamSize();

        visualizer_.reset(
            new Visualizer(pnh_, config_file_, trace_dir, cam_size));

    }

    SvoProcess::~SvoProcess()
    {
        if (imu_thread_)
            imu_thread_->join();
        if (image_thread_)
            image_thread_->join();
        if (odom_thread_)
            odom_thread_->join();

        ROS_INFO_STREAM("Destructed SVO.");
    }

    void SvoProcess::publishResults(
        const std::vector<cv::Mat> &images,
        const int64_t timestamp_nanoseconds)
    {
        // CHECK_NOTNULL(svo_interface_.get());
        //CHECK_NOTNULL(visualizer_.get());

        //visualizer_->img_caption_.clear();

        int stage = MiVins_GetStage();
        std::cout << "publishResults stage = " << stage << std::endl;

        // int track_qulity = svo_interface_->getTrackingQuality();
        // size_t last_obs_num = svo_interface_->getLastNumObservations();
        // double last_process_time =  svo_interface_->getLastProcessingTime();

        // visualizer_->publishSvoInfo(timestamp_nanoseconds, stage,
        //           track_qulity, last_obs_num, last_process_time);

        // switch (svo_->stage())
        switch (stage)
        {
        case 2: // Stage::kTracking:
        {
            Eigen::Matrix<double, 6, 6> covariance;
            covariance.setZero();
            // bool last_is_kf = svo_interface_->isLastKeyFrame();
            // int64_t last_frame_ts = svo_interface_->GetLastFramesTimestamp();

            // std::vector<int> last_frame_ids = svo_interface_->getLastFramesId();
            //Transformation last_imu_pose = svo_interface_->getLastFramesImuPose();
            // std::vector<cv::Mat> last_frame_imgs = svo_interface_->getLastFramesImage();
            // std::vector<Eigen::MatrixXd> last_frame_pts = svo_interface_->getLastFramesPts();
            //std::vector<std::vector<double>> last_cam_poses = MiVins_GetLastFramesCamPose();
            //std::cout << "publishResults last_cam_poses.size = " << static_cast<int>(last_cam_poses.size()) << " Pxyz = [ " << last_cam_poses[0][0] << "," << last_cam_poses[0][1] << "," << last_cam_poses[0][2] << " ]" << std::endl;
            const std::string sensor_type = "cam";
            std::vector<std::vector<double>> last_cam_poses = MiVins_GetLastFramesPose(sensor_type);
            if(last_cam_poses.empty())
              break;
            
            std::cout << "publishResults last_cam_poses.size = " 
                      << static_cast<int>(last_cam_poses.size()) 
                       << " Pxyz = [ " << last_cam_poses[0][0] << "," 
                       << last_cam_poses[0][1] << "," 
                       << last_cam_poses[0][2] << " ]" << std::endl;
            // std::vector<Transformation> active_kf_poses = svo_interface_->getActiveKeyframePoses();
            // std::vector<Eigen::Vector4d> map_region_pts = svo_interface_->getMapRegionPoints();

            // std::vector<Eigen::Vector3d> map_seeds = svo_interface_->getMapSeeds();
            // std::vector<Eigen::Vector3d> map_seeds_uncertainty = svo_interface_->getMapSeedsUncertainty();

            // std::unordered_map<int, Transformation> close_kf_poses = svo_interface_->getCloseKfPoses();
            // std::unordered_map<int, Eigen::Vector4d> close_kf_all_pts = svo_interface_->getCloseKfPts();

            //visualizer_->publishImuPose(last_imu_pose, covariance, timestamp_nanoseconds);
            visualizer_->publishCameraPoses(last_cam_poses, timestamp_nanoseconds);
            visualizer_->publishOdometry(last_cam_poses, timestamp_nanoseconds);
            // visualizer_->visualizeMarkers(last_cam_poses, active_kf_poses, close_kf_poses, map_region_pts,
            //       map_seeds, map_seeds_uncertainty, close_kf_all_pts, last_frame_ts, last_is_kf);
            // visualizer_->exportToDense(last_frame_ts, last_frame_ids,
            //         last_frame_imgs, last_cam_poses, last_frame_pts);

            // bool draw_boundary = false;
            // for(size_t i = 0; i < last_frame_ids.size(); ++i)
            // {
            //   std::vector<cv::Mat> frame_pyr = svo_interface_->getLastFramePyr(i);
            //   Eigen::MatrixXd feat_px = svo_interface_->getLastFrameFeaturePx(i);
            //   Eigen::MatrixXd feat_grad = svo_interface_->getLastFrameFeatureGrad(i);
            //   Eigen::MatrixXi feat_status = svo_interface_->getLastFrameFeatureStatus(i);

            //   visualizer_->publishImagesWithFeatures(i, last_frame_ts,
            //             frame_pyr, feat_px, feat_grad, feat_status, draw_boundary);
            // }

            // #ifdef SVO_LOOP_CLOSING
            /*
      // detections
      if (svo_->lc_)
      {
        visualizer_->publishLoopClosureInfo(
              svo_->lc_->cur_loop_check_viz_info_,
              std::string("loop_query"),
              Eigen::Vector3f(0.0f, 0.0f, 1.0f), 0.5);
        visualizer_->publishLoopClosureInfo(
              svo_->lc_->loop_detect_viz_info_, std::string("loop_detection"),
              Eigen::Vector3f(1.0f, 0.0f, 0.0f), 1.0);
        if (svo_->IsBackendValid())
        {
          visualizer_->publishLoopClosureInfo(
                svo_->lc_->loop_correction_viz_info_,
                std::string("loop_correction"),
                Eigen::Vector3f(0.0f, 1.0f, 0.0f), 3.0);
        }
        if (svo_->GetLastFrames()->at(0)->IsKeyframe())
        {
          bool pc_recalculated = visualizer_->publishPoseGraph(
                svo_->lc_->kf_list_,
                svo_->lc_->need_to_update_pose_graph_viz_,
                static_cast<size_t>(svo_->lc_->options_.ignored_past_frames));
          if(pc_recalculated)
          {
            svo_->lc_->need_to_update_pose_graph_viz_ = false;
          }
        }
      }
      */
            // #endif
            break;
        }
        case 1: // Stage::kInitializing:
        {
            // visualizer_->publishBundleFeatureTracks(
            //       svo_->initializer_->frames_ref_, svo_->GetLastFrames(),
            //       timestamp_nanoseconds);
            break;
        }
        case 0: // Stage::kPaused:
        case 3: // Stage::kRelocalization:
        {
            // for(size_t i = 0; i < images.size(); ++i)
            // {
            //   std::vector<cv::Mat> frame_pyr = svo_interface_->getNewFramePyr(i);
            //   visualizer_->publishImages(i, frame_pyr, timestamp_nanoseconds);
            // }
            break;
        }
        default:
            //LOG(FATAL) << "Unknown stage";
            break;
        }
    }

    void SvoProcess::rgbdCallback(
        const sensor_msgs::ImageConstPtr &msg_image,
        const sensor_msgs::ImageConstPtr &msg_depth)
    {
        cv::Mat image, depth;
        try
        {
            image = cv_bridge::toCvShare(msg_image, "mono8")->image;
            depth = cv_bridge::toCvShare(msg_depth, "16UC1")->image;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        std::vector<cv::Mat> images;
        std::map<int, cv::Mat> depths;
        images.emplace_back(image);

        depths[0] = depth;

        const int64_t ts = msg_image->header.stamp.toNSec();
        MiVins_ImgInput(ts, images, depths);
        //svo_interface_->processImageData(ts, images, depths);
        publishResults(images, ts);
    }

    void SvoProcess::monoCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        if (idle_)
            return;

        cv::Mat image;
        try
        {
            image = cv_bridge::toCvCopy(msg)->image;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        std::vector<cv::Mat> images;
        std::map<int, cv::Mat> depths;
        images.push_back(image.clone());

        const int64_t ts = msg->header.stamp.toNSec();
        MiVins_ImgInput(ts, images, depths);
        //svo_interface_->processImageData(ts, images, depths);
        //publishResults(images, ts);
    }

    void SvoProcess::stereoCallback(
        const sensor_msgs::ImageConstPtr &msg0,
        const sensor_msgs::ImageConstPtr &msg1)
    {
        if (idle_)
            return;

        cv::Mat img0, img1;
        try
        {
            img0 = cv_bridge::toCvShare(msg0, "mono8")->image;
            img1 = cv_bridge::toCvShare(msg1, "mono8")->image;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        //int64 start_time = cv::getTickCount();
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(img0, img0);
        clahe->apply(img1, img1);
        //int64 end_time = cv::getTickCount();
        //printf("CLAHE Time cost --------------------------------------------------------------------------------------------------------------------TIMER_ : %.2fms\n", 1000.*((end_time - start_time) / cv::getTickFrequency()));
        std::vector<cv::Mat> images;
        std::map<int, cv::Mat> depths;
        images.emplace_back(img0);
        images.emplace_back(img1);

        const int64_t ts = msg0->header.stamp.toNSec();
        MiVins_ImgInput(ts, images, depths);
        //svo_interface_->processImageData(ts, images, depths);
        publishResults(images, ts);
    }

    void SvoProcess::tripleWithStereoCallback(
        const sensor_msgs::ImageConstPtr &msg0,
        const sensor_msgs::ImageConstPtr &msg1,
        const sensor_msgs::ImageConstPtr &msg2)
    {
        if (idle_)
            return;

        cv::Mat img0, img1, img2;
        try
        {
            img0 = cv_bridge::toCvShare(msg0, "mono8")->image;
            img1 = cv_bridge::toCvShare(msg1, "mono8")->image;
            img2 = cv_bridge::toCvShare(msg2, "mono8")->image;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        std::vector<cv::Mat> images;
        std::map<int, cv::Mat> depths;
        images.emplace_back(img0);
        images.emplace_back(img1);
        images.emplace_back(img2);

        const int64_t ts = msg0->header.stamp.toNSec();
        MiVins_ImgInput(ts, images, depths);
        // svo_interface_->processImageData(ts, images, depths);
        // publishResults(images, ts);
    }

    void SvoProcess::tripleWithDepthCallback(
        const sensor_msgs::ImageConstPtr &msg_front,
        const sensor_msgs::ImageConstPtr &msg_left,
        const sensor_msgs::ImageConstPtr &msg_right,
        const sensor_msgs::ImageConstPtr &msg_depth)
    {
        cv::Mat img_front, img_left, img_right, img_depth;
        try
        {
            img_front = cv_bridge::toCvShare(msg_front, "mono8")->image;
            img_left = cv_bridge::toCvShare(msg_left, "mono8")->image;
            img_right = cv_bridge::toCvShare(msg_right, "mono8")->image;
            img_depth = cv_bridge::toCvShare(msg_depth, "16UC1")->image;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        std::vector<cv::Mat> images;
        std::map<int, cv::Mat> depths;
        images.emplace_back(img_front);
        images.emplace_back(img_left);
        images.emplace_back(img_right);

        depths[0] = img_depth;

        const int64_t ts = msg_front->header.stamp.toNSec();
        MiVins_ImgInput(ts, images, depths);
        // svo_interface_->processImageData(ts, images, depths);
        publishResults(images, ts);
    }

    void SvoProcess::imuCallback(const sensor_msgs::ImuConstPtr &msg)
    {
        double ts = msg->header.stamp.toSec();
        const Eigen::Vector3d omega_imu(
            msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        const Eigen::Vector3d lin_acc_imu(
            msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        MiVins_ImuInput(ts, lin_acc_imu, omega_imu);
        // svo_interface_->inputImuData(ts, lin_acc_imu, omega_imu);
    }

    void SvoProcess::odomCallback(const nav_msgs::OdometryConstPtr &odom_msg)
    {
        double ts = odom_msg->header.stamp.toSec();
        
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
        //svo_interface_->inputOdomData(ts, linear_velocity, angular_velocity);
    }

    void SvoProcess::inputKeyCallback(const std_msgs::StringConstPtr &key_input)
    {
        std::string remote_input = key_input->data;
        char input = remote_input.c_str()[0];
        // switch(input)
        // {
        //   case 'q':
        //     quit_ = true;
        //     SVO_INFO_STREAM("SVO user input: QUIT");
        //     break;
        //   case 'r':
        //     svo_->reset();
        //     idle_ = true;
        //     SVO_INFO_STREAM("SVO user input: RESET");
        //     break;
        //   case 's':
        //     svo_->start();
        //     idle_ = false;
        //     SVO_INFO_STREAM("SVO user input: START");
        //     break;
        //    case 'c':
        //     svo_->SetCompensation(true);
        //     SVO_INFO_STREAM("Enabled affine compensation.");
        //     break;
        //    case 'C':
        //     svo_->SetCompensation(false);
        //     SVO_INFO_STREAM("Disabled affine compensation.");
        //     break;
        //   default: ;
        // }
    }

    void SvoProcess::subscribeImu()
    {
        imu_thread_ = std::unique_ptr<std::thread>(
            new std::thread(&SvoProcess::imuLoop, this));
        sleep(3);
    }

    void SvoProcess::subscribeOdom()
    {
        odom_thread_ = std::unique_ptr<std::thread>(
            new std::thread(&SvoProcess::odomLoop, this));
        sleep(3);
    }

    void SvoProcess::subscribeImage()
    {
        int pipeline_type = MiVins_GetPipelineType();
        std::cout << "SvoProcess::subscribeImage() pipeline_type = " << pipeline_type << std::endl;
        if (pipeline_type == 0) // PipelineType::kRgbd)
            image_thread_ = std::unique_ptr<std::thread>(
                new std::thread(&SvoProcess::rgbdLoop, this));
        else if (pipeline_type == 1) // == PipelineType::kMono)
            image_thread_ = std::unique_ptr<std::thread>(
                new std::thread(&SvoProcess::monoLoop, this));
        else if (pipeline_type == 2) // PipelineType::kStereo)
            image_thread_ = std::unique_ptr<std::thread>(
                new std::thread(&SvoProcess::stereoLoop, this));
        else if (pipeline_type == 3) // PipelineType::kTripleWithStereo)
            image_thread_ = std::unique_ptr<std::thread>(
                new std::thread(&SvoProcess::tripleWithStereoLoop, this));
        else if (pipeline_type == 4) // PipelineType::kTripleWithDepth)
            image_thread_ = std::unique_ptr<std::thread>(
                new std::thread(&SvoProcess::tripleWithDepthLoop, this));
    }

    void SvoProcess::subscribeRemoteKey()
    {
        // sub_remote_key_ =
        //     nh_.subscribe(remote_key_topic_, 5, &mivins::SvoProcess::inputKeyCallback, this);
    }

    void SvoProcess::imuLoop()
    {
        ROS_INFO_STREAM("SvoNode: Started IMU loop.");
        ros::NodeHandle nh;
        ros::CallbackQueue queue;
        nh.setCallbackQueue(&queue);

        std::string imu_topic;
        pnh_.param<std::string>("imu_topic", imu_topic, "imu");

        ros::Subscriber sub_imu =
            nh.subscribe(imu_topic, 10, &mivins::SvoProcess::imuCallback, this);
        while (ros::ok() && !quit_)
        {
            queue.callAvailable(ros::WallDuration(0.1));
        }
    }

    void SvoProcess::odomLoop()
    {
        ROS_INFO_STREAM("SvoNode: Started Odom loop.");
        ros::NodeHandle nh;
        ros::CallbackQueue queue;
        nh.setCallbackQueue(&queue);

        std::string odom_topic;
        pnh_.param<std::string>("odom_topic", odom_topic, "/odom");

        ros::Subscriber sub_odom =
            nh.subscribe(odom_topic, 10, &mivins::SvoProcess::odomCallback, this);
        while (ros::ok() && !quit_)
        {
            queue.callAvailable(ros::WallDuration(0.1));
        }
    }

    void SvoProcess::rgbdLoop()
    {
        // ExactTime is more strict than ApproximateTime. The timestamp of stereo in dog is not alligned
        // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ExactPolicy;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ExactPolicy;
        typedef message_filters::Synchronizer<ExactPolicy> ExactSync;

        ros::NodeHandle nh(nh_, "image_thread");
        ros::CallbackQueue queue;
        nh.setCallbackQueue(&queue);

        // subscribe to cam msgs
        std::string topic_image, topic_depth;
        pnh_.param<std::string>("image_topic", topic_image, "/front/image_raw");
        pnh_.param<std::string>("depth_topic", topic_depth, "/depth/image_raw");

        image_transport::ImageTransport it(nh);
        image_transport::SubscriberFilter sub_image(it, topic_image, 1, std::string("raw"));
        image_transport::SubscriberFilter sub_depth(it, topic_depth, 1, std::string("raw"));

        // ExactSync sync_sub(ExactPolicy(5), sub0, sub1);
        ExactSync sync_sub(ExactPolicy(15), sub_image, sub_depth);
        sync_sub.registerCallback(boost::bind(&mivins::SvoProcess::rgbdCallback, this, _1, _2));

        while (ros::ok() && !quit_)
        {
            queue.callAvailable(ros::WallDuration(0.1));
        }
    }

    void SvoProcess::monoLoop()
    {
        ROS_INFO_STREAM("SvoNode: Started Image loop.");

        ros::NodeHandle nh;
        ros::CallbackQueue queue;
        nh.setCallbackQueue(&queue);

        image_transport::ImageTransport it(nh);

        std::string image_topic;
        pnh_.param<std::string>("cam0_topic", image_topic, "camera/image_raw");

        image_transport::Subscriber it_sub =
            it.subscribe(image_topic, 5, &mivins::SvoProcess::monoCallback, this);

        while (ros::ok() && !quit_)
        {
            queue.callAvailable(ros::WallDuration(0.1));
        }
    }

    void SvoProcess::stereoLoop()
    {
        // ExactTime is more strict than ApproximateTime. The timestamp of stereo in dog is not alligned
        // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ExactPolicy;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ExactPolicy;
        typedef message_filters::Synchronizer<ExactPolicy> ExactSync;

        ros::NodeHandle nh(nh_, "image_thread");
        ros::CallbackQueue queue;
        nh.setCallbackQueue(&queue);

        // subscribe to cam msgs
        std::string cam0_topic, cam1_topic;
        pnh_.param<std::string>("cam0_topic", cam0_topic, "/cam0/image_raw");
        pnh_.param<std::string>("cam1_topic", cam1_topic, "/cam1/image_raw");

        image_transport::ImageTransport it(nh);
        image_transport::SubscriberFilter sub0(it, cam0_topic, 1, std::string("raw"));
        image_transport::SubscriberFilter sub1(it, cam1_topic, 1, std::string("raw"));
        // ExactSync sync_sub(ExactPolicy(5), sub0, sub1);
        ExactSync sync_sub(ExactPolicy(15), sub0, sub1);
        sync_sub.registerCallback(boost::bind(&mivins::SvoProcess::stereoCallback, this, _1, _2));

        while (ros::ok() && !quit_)
        {
            queue.callAvailable(ros::WallDuration(0.1));
        }
    }

    void mivins::SvoProcess::tripleWithStereoLoop()
    {
        // ExactTime is more strict than ApproximateTime. The timestamp of stereo in dog is not alligned
        // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ExactPolicy;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                                sensor_msgs::Image, sensor_msgs::Image>
            ExactPolicy;
        typedef message_filters::Synchronizer<ExactPolicy> ExactSync;

        ros::NodeHandle nh(nh_, "image_thread");
        ros::CallbackQueue queue;
        nh.setCallbackQueue(&queue);

        // subscribe to cam msgs
        std::string cam0_topic, cam1_topic, cam2_topic;
        pnh_.param<std::string>("cam0_topic", cam0_topic, "/cam0/image_raw");
        pnh_.param<std::string>("cam1_topic", cam1_topic, "/cam1/image_raw");
        pnh_.param<std::string>("cam2_topic", cam2_topic, "/cam2/image_raw");

        image_transport::ImageTransport it(nh);
        image_transport::SubscriberFilter sub0(it, cam0_topic, 1, std::string("raw"));
        image_transport::SubscriberFilter sub1(it, cam1_topic, 1, std::string("raw"));
        image_transport::SubscriberFilter sub2(it, cam2_topic, 1, std::string("raw"));

        // ExactSync sync_sub(ExactPolicy(5), sub0, sub1);
        ExactSync sync_sub(ExactPolicy(15), sub0, sub1, sub2);
        sync_sub.registerCallback(boost::bind(&mivins::SvoProcess::tripleWithStereoCallback, this, _1, _2, _3));

        while (ros::ok() && !quit_)
        {
            queue.callAvailable(ros::WallDuration(0.1));
        }
    }

    void mivins::SvoProcess::tripleWithDepthLoop()
    {
        // ExactTime is more strict than ApproximateTime. The timestamp of stereo in dog is not alligned
        // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ExactPolicy;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                                sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image>
            ExactPolicy; //
        typedef message_filters::Synchronizer<ExactPolicy> ExactSync;

        ros::NodeHandle nh(nh_, "image_thread");
        ros::CallbackQueue queue;
        nh.setCallbackQueue(&queue);

        // subscribe to cam msgs
        std::string topic_front, topic_left, topic_right, topic_depth;
        pnh_.param<std::string>("front_topic", topic_front, "/front/image_raw");
        pnh_.param<std::string>("left_topic", topic_left, "/left/image_raw");
        pnh_.param<std::string>("right_topic", topic_right, "/right/image_raw");
        pnh_.param<std::string>("depth_topic", topic_depth, "/depth/image_raw");

        image_transport::ImageTransport it(nh);
        image_transport::SubscriberFilter sub_front(it, topic_front, 1, std::string("raw"));
        image_transport::SubscriberFilter sub_left(it, topic_left, 1, std::string("raw"));
        image_transport::SubscriberFilter sub_right(it, topic_right, 1, std::string("raw"));
        image_transport::SubscriberFilter sub_depth(it, topic_depth, 1, std::string("raw"));

        // ExactSync sync_sub(ExactPolicy(5), sub0, sub1);
        ExactSync sync_sub(ExactPolicy(15), sub_front, sub_left, sub_right, sub_depth);                             //
        sync_sub.registerCallback(boost::bind(&mivins::SvoProcess::tripleWithDepthCallback, this, _1, _2, _3, _4)); //

        while (ros::ok() && !quit_)
        {
            queue.callAvailable(ros::WallDuration(0.1));
        }
    }

} // namespace mivins
