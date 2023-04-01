#pragma once

#include <rclcpp/serialized_message.hpp>
#include <string>

#include "pipeline/action_interface.hpp"

namespace dtnproxy::pipeline {

class ImageDecompressionAction : public IAction {
private:
    static constexpr auto supportedMsgType = "sensor_msgs/msg/Image";
    static constexpr auto BIT_DEPTH = 8;
    static constexpr auto ENCODING = "rgb8";
    const uint SEQUENCE_NR = 1;
    const Direction dir = Direction::OUT;

    bool active = true;
    rclcpp::SerializedMessage serializedMsg;

public:
    ImageDecompressionAction(const std::string &msgType);

    Direction direction() override;
    uint order() override;
    bool run(std::shared_ptr<rclcpp::SerializedMessage> msg) override;
};

}  // namespace dtnproxy::pipeline
