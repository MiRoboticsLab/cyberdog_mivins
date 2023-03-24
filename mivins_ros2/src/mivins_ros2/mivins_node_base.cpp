#include "mivins_node_base.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

namespace mivins_ros2
{
    MivinsNodeBase::MivinsNodeBase()
        : Node("mivins_node")
    {
        mivins_process_.reset(new mivins::MivinsProcess(this));
        if (MiVins_GetImuStatus())
        {
            mivins_process_->subscribeImu();
        }

        if (MiVins_GetOdomStatus())
        {
            mivins_process_->subscribeOdom();
        }

        mivins_process_->subscribeImage();
        mivins_process_->subscribeRemoteKey();

        mivins_process_->subscribeReloc();
        mivins_process_->subscribeMapServer();
    }

}
