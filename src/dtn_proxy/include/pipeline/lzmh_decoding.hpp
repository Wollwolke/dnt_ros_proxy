#pragma once

extern "C" {
#include "enc_dec.h"
#include "file_buffer.h"
}

#include "pipeline/action_interface.hpp"

namespace dtnproxy::pipeline {

class LzmhDecodingAction : public IAction {
private:
    static constexpr auto BITS_IN_BYTE = 8;
    const uint SEQUENCE_NR = 1;
    const Direction dir = Direction::OUT;

    file_buffer_t* fbIn;
    file_buffer_t* fbOut;
    options_t options;

public:
    LzmhDecodingAction();
    ~LzmhDecodingAction();
    LzmhDecodingAction(const LzmhDecodingAction&) = delete;
    LzmhDecodingAction& operator=(const LzmhDecodingAction&) = delete;

    Direction direction() override;
    uint order() override;
    bool run(std::shared_ptr<rclcpp::SerializedMessage> msg) override;
};

}  // namespace dtnproxy::pipeline
