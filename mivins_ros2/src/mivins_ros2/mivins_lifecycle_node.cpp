#include "mivins_lifecycle.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    LOGGER_MAIN_INSTANCE("vins"); 
    std::shared_ptr<mivins_ros2::MIVINSLIFECYCLE> node_handle_ =
      std::make_shared<mivins_ros2::MIVINSLIFECYCLE>();
    rclcpp::spin(node_handle_->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}

