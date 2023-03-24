#pragma once

#include "svo_process.h"

namespace svo_ros
{

    class SvoNodeBase
    {
    public:
        // Initializes glog, gflags and ROS.
        static void initThirdParty(int argc, char **argv);

        SvoNodeBase();
        virtual ~SvoNodeBase();
        void run();

    private:
        ros::NodeHandle node_handle_;
        ros::NodeHandle private_node_handle_;
        //mivins::PipelineType type_;

    public:
        mivins::SvoProcess svo_process_;
    };

} // namespace svo_ros
