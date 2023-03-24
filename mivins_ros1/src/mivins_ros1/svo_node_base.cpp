#include "svo_node_base.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

namespace svo_ros
{

    void SvoNodeBase::initThirdParty(int argc, char **argv)
    {
        // google::InitGoogleLogging(argv[0]);
        // google::ParseCommandLineFlags(&argc, &argv, true);
        // google::InstallFailureSignalHandler();

        ros::init(argc, argv, "svo");
    }

    SvoNodeBase::SvoNodeBase()
        : node_handle_(), private_node_handle_("~"),
          svo_process_(node_handle_, private_node_handle_)
    {
        if (MiVins_GetImuStatus())
        {
            svo_process_.subscribeImu();
        }

        if (MiVins_GetOdomStatus())
        {
            svo_process_.subscribeOdom();
        }

        svo_process_.subscribeImage();
        svo_process_.subscribeRemoteKey();
    }
    SvoNodeBase::~SvoNodeBase()
    {
    }
    void SvoNodeBase::run()
    {
        ros::spin();
        ROS_INFO_STREAM("SVO quit");
        svo_process_.quit_ = true;
        ROS_INFO_STREAM("SVO terminated.\n");
    }

} // namespace svo_ros
