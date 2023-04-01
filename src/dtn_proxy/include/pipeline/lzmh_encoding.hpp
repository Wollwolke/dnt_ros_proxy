#pragma once

extern "C" {
#include "enc_dec.h"
#include "file_buffer.h"
}

#include <string>

#include "pipeline/action_interface.hpp"

namespace dtnproxy::pipeline {

class LzmhEncodingAction : public IAction {
private:
    static constexpr auto unSupportedMsgType = "sensor_msgs/msg/Image";
    static constexpr auto BITS_IN_BYTE = 8;
    const uint SEQUENCE_NR = 100;
    const Direction dir = Direction::IN;

    bool active = true;
    file_buffer_t* fbIn;
    file_buffer_t* fbOut;
    options_t options;

public:
    LzmhEncodingAction(const std::string& msgType);
    ~LzmhEncodingAction();
    LzmhEncodingAction(const LzmhEncodingAction&) = delete;
    LzmhEncodingAction& operator=(const LzmhEncodingAction&) = delete;

    Direction direction() override;
    uint order() override;
    bool run(std::shared_ptr<rclcpp::SerializedMessage> msg) override;
};

}  // namespace dtnproxy::pipeline
