#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/serialization.hpp>
#include <string>
#include <vector>

#include "pipeline/action_interface.hpp"
#include "pipeline/pipeline_msg.hpp"

namespace dtnproxy::pipeline {

class SplitTfAction : public IAction {
private:
    // TODO: figure out the correct sequence for all modules
    const uint SEQUENCE_NR = 5;
    const Direction dir = Direction::OUT;

    rclcpp::Serialization<geometry_msgs::msg::TransformStamped> tfSerialization;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

public:
    SplitTfAction(rclcpp::Node& nodeHandle);

    Direction direction() override;
    uint order() override;
    bool run(PipelineMessage& pMsg) override;
};

}  // namespace dtnproxy::pipeline
