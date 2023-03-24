#include "mivins_lifecycle.h"

#include <gflags/gflags.h>
// #include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

namespace mivins_ros2
{
    MIVINSLIFECYCLE::MIVINSLIFECYCLE()
        : nav2_util::LifecycleNode("map_mode_node","",true)
    {
        init_mivins_ = false;
        declare_parameter("mode_type", rclcpp::ParameterValue(0));
        declare_parameter("cam0_topic", rclcpp::ParameterValue(""));
        declare_parameter("cam1_topic", rclcpp::ParameterValue(""));
        declare_parameter("imu_topic", rclcpp::ParameterValue(""));
        declare_parameter("odom_topic", rclcpp::ParameterValue(""));
        declare_parameter("status_topic", rclcpp::ParameterValue(""));
        declare_parameter("calib_file", rclcpp::ParameterValue(""));
        declare_parameter("config_file", rclcpp::ParameterValue(""));
        declare_parameter("mode_stable", rclcpp::ParameterValue(0));
        declare_parameter("reloc_waittime", rclcpp::ParameterValue(20));
        declare_parameter("map_waittime", rclcpp::ParameterValue(10));
        declare_parameter("pose_path", rclcpp::ParameterValue(""));
    }

    MIVINSLIFECYCLE::~MIVINSLIFECYCLE() {}

    nav2_util::CallbackReturn MIVINSLIFECYCLE::on_configure(
        const rclcpp_lifecycle::State& state) 
    {
        INFO("[MIVINS][ON_CONFIGURE] is called.");
        this->get_parameter("mode_type", mode_type_);
        this->get_parameter("cam0_topic", cam0_topic_);
        this->get_parameter("cam1_topic", cam1_topic_);
        this->get_parameter("imu_topic", imu_topic_);
        this->get_parameter("odom_topic", odom_topic_);
        this->get_parameter("status_topic", status_topic_);
        this->get_parameter("calib_file", calib_file_);
        this->get_parameter("config_file", config_file_);
        this->get_parameter("mode_stable", mode_stable_);
        this->get_parameter("reloc_waittime", reloc_waittime_);
        this->get_parameter("map_waittime", map_waittime_);
        this->get_parameter("pose_path", pose_path_);
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn MIVINSLIFECYCLE::on_activate(
        const rclcpp_lifecycle::State& state) 
    {
        if(!init_mivins_) {
            mivins_process_.reset(new mivins::MivinsProcess(this->rclcpp_node_,calib_file_,config_file_));
            init_mivins_ = true;
        }

        mivins_process_->initMivins();
        // if(mode_type_ == 1 || mode_type_ == 3) {
        //     mivins_process_->start_map_ = true;
        // }
        if(mode_type_ == 3)
            mivins_process_->start_map_ = true;
        if(mode_type_ == 1)
        {
            mivins_process_->savefile();
            remove(pose_path_.c_str());
            mivins_process_->save_twb.open(pose_path_); 
        }   
        mivins_process_->mivins_mode_ = mode_type_;//1 mapping ;2 localization_navigation
        mivins_process_->cam0_topic_ = cam0_topic_;
        mivins_process_->cam1_topic_ = cam1_topic_;
        mivins_process_->imu_topic_ = imu_topic_;
        mivins_process_->odom_topic_ = odom_topic_;
        mivins_process_->status_topic_ = status_topic_;
        mivins_process_->mivins_mode_stable_ = mode_stable_;
        INFO_STREAM("[MIVINS][ON_ACTIVATE]mode type: " << mode_type_);
        INFO_STREAM("[MIVINS][ON_ACTIVATE]stable mode type: " << mode_stable_);
        INFO_STREAM("[MIVINS][ON_ACTIVATE]reloc_waittime: " << reloc_waittime_);
        INFO("[MIVINS][ON_ACTIVATE] is called.");
        if(mode_type_ == 1)//mivins_process_->subscribeMapServer();
        {
            mivins_process_->createWaitMap();
            int time_wait = 0;
            int time_threshold = map_waittime_;
            while(!mivins_process_->miloc_start_map && time_wait < map_waittime_) {
                time_wait++;
                INFO_STREAM("[MIVINS][ON_ACTIVATE] wait for miloc start..." << time_wait);
                sleep(1.0);
            }
            if(!mivins_process_->miloc_start_map) {
                if (mivins_process_->wait_map_thread_) {
                    mivins_process_->wait_map_thread_->join();
                    mivins_process_->wait_map_thread_ = nullptr;
                }
                ERROR("[MIVINS][ON_ACTIVATE]miloc error!");
                createBond();
                return nav2_util::CallbackReturn::FAILURE;
            } else {
                INFO("[MIVINS][ON_ACTIVATE]Get miloc Info: start success!");
            }

            mivins_process_->subscribeMapServer();

        }    
        else if(mode_type_ == 2)
        {
            mivins_process_->createWaitRelc();
            int time_threshold = reloc_waittime_;//25max waiting time
            int time_wait = 0;
            while(!mivins_process_->b_miloc_get_info_ && time_wait < time_threshold) {
                time_wait++;
                INFO_STREAM("[MIVINS][ON_ACTIVATE] wait for reloc start..." << time_wait);
                sleep(1.0);
            }
            if(!mivins_process_->b_miloc_run_success_) {
                ERROR("[MIVINS][ON_ACTIVATE]Get Reloc Info: start failed!");
                mivins_process_->b_comeoutofserver_ = true;
                if (mivins_process_->wait_reloc_thread_) {
                    if(mivins_process_->wait_reloc_thread_->joinable()) {
                        mivins_process_->wait_reloc_thread_->join();
                    }
                }
                ERROR("[MIVINS][ON_ACTIVATE]kill Reloc server,Please Check Miloc exe, and please Lifecycle re_activate!");
                createBond();
                return nav2_util::CallbackReturn::FAILURE;
            } else {
                INFO("[MIVINS][ON_ACTIVATE]Get Reloc Info: start success!");
            }
            mivins_process_->subscribeReloc();
            mivins_process_->subscribeNavigationServer();
        }
        else if(mode_type_ == 3)
        {
            mivins_process_->activateMivins();
        }

        createBond();
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn MIVINSLIFECYCLE::on_deactivate(
        const rclcpp_lifecycle::State& state) 
    {
        INFO("[MIVINS][ON_DEACTIVATE] is called.");
        if(mode_type_ == 3)
        {
            mivins_process_->deactivateMivins();
            mivins_process_->cleanupMivins();
        }
       
        destroyBond();
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn MIVINSLIFECYCLE::on_cleanup(
    const rclcpp_lifecycle::State& state) 
    {
        INFO("[MIVINS][ON_CLEANUP] is called.Lifecycle end!");
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn MIVINSLIFECYCLE::on_shutdown(
        const rclcpp_lifecycle::State& state) 
    {
        INFO("[MIVINS][ON_SHUTDOWN] is called.");
        return nav2_util::CallbackReturn::SUCCESS;
    }
}
