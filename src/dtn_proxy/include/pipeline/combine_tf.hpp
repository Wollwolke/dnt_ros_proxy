#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cstdint>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/serialization.hpp>
#include <string>
#include <vector>

#include "pipeline/action_interface.hpp"
#include "pipeline/pipeline_msg.hpp"

namespace dtnproxy::pipeline {

class CombineTfAction : public IAction {
private:
    // TODO: figure out the correct sequence for all modules
    const uint SEQUENCE_NR = 85;
    const Direction dir = Direction::IN;

    std::string sourceFrame;
    std::string targetFrame;

    rclcpp::Serialization<geometry_msgs::msg::TransformStamped> tfSerialization;
    std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;

    static void appendMessage(std::vector<uint8_t>& buffer, rclcpp::SerializedMessage& msg,
                              bool empty = false);

public:
    CombineTfAction(std::vector<std::string> params, rclcpp::Node& nodeHandle);

    Direction direction() override;
    uint order() override;
    bool run(PipelineMessage& pMsg) override;
};

}  // namespace dtnproxy::pipeline
