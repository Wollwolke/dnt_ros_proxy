#pragma once

extern "C" {
#include "enc_dec.h"
#include "file_buffer.h"
}

#include "pipeline/action_interface.hpp"

namespace dtnproxy::pipeline {

class LzmhEncodingAction : public IAction {
private:
    const uint SEQUENCE_NR = 100;
    const Direction dir = Direction::IN;

    file_buffer_t* fbIn;
    file_buffer_t* fbOut;
    options_t options;

public:
    LzmhEncodingAction();
    ~LzmhEncodingAction();
    LzmhEncodingAction(const LzmhEncodingAction&) = delete;
    LzmhEncodingAction& operator=(const LzmhEncodingAction&) = delete;

    Direction direction() override;
    uint order() override;
    bool run(std::shared_ptr<rclcpp::SerializedMessage> msg) override;
};

}  // namespace dtnproxy::pipeline
