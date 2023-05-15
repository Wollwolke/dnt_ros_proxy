#pragma once

#include <memory>
#include <rclcpp/serialized_message.hpp>
#include <string>
#include <vector>

#include "pipeline/action_interface.hpp"

namespace dtnproxy::pipeline {

class ImageCompressionAction : public IAction {
private:
    static constexpr auto supportedMsgType = "sensor_msgs/msg/Image";
    static constexpr auto BIT_DEPTH = 8;
    const uint SEQUENCE_NR = 80;
    const Direction dir = Direction::IN;

    bool active = true;

public:
    ImageCompressionAction(const std::string &msgType);

    Direction direction() override;
    uint order() override;
    bool run(PipelineMessage& pMsg) override;
};

}  // namespace dtnproxy::pipeline
