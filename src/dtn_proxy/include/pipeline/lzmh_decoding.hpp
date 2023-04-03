#pragma once

extern "C" {
#include "enc_dec.h"
#include "file_buffer.h"
}

#include <rclcpp/serialized_message.hpp>
#include <string>

#include "pipeline/action_interface.hpp"

namespace dtnproxy::pipeline {

class LzmhDecodingAction : public IAction {
private:
    static constexpr auto unSupportedMsgType = "sensor_msgs/msg/Image";
    static constexpr auto BITS_IN_BYTE = 8;
    const uint SEQUENCE_NR = 1;
    const Direction dir = Direction::OUT;

    bool active = true;
    file_buffer_t* fbIn;
    file_buffer_t* fbOut;
    options_t options;

public:
    LzmhDecodingAction(const std::string& msgType);
    ~LzmhDecodingAction();
    LzmhDecodingAction(const LzmhDecodingAction&) = delete;
    LzmhDecodingAction& operator=(const LzmhDecodingAction&) = delete;

    Direction direction() override;
    uint order() override;
    bool run(PipelineMessage& pMsg) override;
};

}  // namespace dtnproxy::pipeline
