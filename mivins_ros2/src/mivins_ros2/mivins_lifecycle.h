#pragma once
#include "mivins_process.h"
#include "nav2_util/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace mivins_ros2
{
    class MIVINSLIFECYCLE : public nav2_util::LifecycleNode
    {
        public:
            MIVINSLIFECYCLE();
            virtual ~MIVINSLIFECYCLE();
        protected:
            nav2_util::CallbackReturn on_configure(
                const rclcpp_lifecycle::State& state) override;

            nav2_util::CallbackReturn on_activate(
                const rclcpp_lifecycle::State& state) override;

            nav2_util::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State& state) override;

            nav2_util::CallbackReturn on_cleanup(
                const rclcpp_lifecycle::State& state) override;

            nav2_util::CallbackReturn on_shutdown(
                const rclcpp_lifecycle::State& state) override;

        private:
            mivins::PipelineType type_;
            rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_mapping_service_;
            rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stop_mapping_service_;

        public:
            std::unique_ptr<mivins::MivinsProcess> mivins_process_;
            int mode_type_;
            std::string cam0_topic_;
            std::string cam1_topic_;
            std::string imu_topic_;
            std::string odom_topic_;
            std::string status_topic_;
            std::string calib_file_;
            std::string config_file_;
            bool init_mivins_;
            int mode_stable_;
            int reloc_waittime_;
            int map_waittime_;
            std::string pose_path_;
    };

} // namespace mivins_ros2
