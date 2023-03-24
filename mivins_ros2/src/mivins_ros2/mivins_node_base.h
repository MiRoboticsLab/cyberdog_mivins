#pragma once
#include "mivins_process.h"
namespace mivins_ros2
{

    class MivinsNodeBase : public rclcpp::Node
    {
    public:
        MivinsNodeBase();
        ~MivinsNodeBase() = default;

    private:
        mivins::PipelineType type_;

    public:
        std::unique_ptr<mivins::MivinsProcess> mivins_process_;
    };

} // namespace mivins_ros2
