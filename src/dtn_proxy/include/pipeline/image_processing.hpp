#pragma once

#include <rclcpp/serialized_message.hpp>
#include <string>

#include "pipeline/action_interface.hpp"

namespace dtnproxy::pipeline {

class ImageProcessingAction : public IAction {
private:
    static constexpr auto supportedMsgType = "sensor_msgs/msg/Image";
    static constexpr auto BIT_DEPTH = 8;
    static constexpr auto ENCODING = "rgb8";
    const uint SEQUENCE_NR_IN = 80;
    const uint SEQUENCE_NR_OUT = 10;
    const Direction dir = Direction::INOUT;

    bool active = true;

    bool compress(PipelineMessage& pMsg);
    bool decompress(PipelineMessage& pMsg);

public:
    ImageProcessingAction(const std::string& msgType);

    Direction direction() override;
    uint order(const Direction& pipelineDir) override;
    bool run(PipelineMessage& pMsg, const Direction& pipelineDir) override;
};

}  // namespace dtnproxy::pipeline
