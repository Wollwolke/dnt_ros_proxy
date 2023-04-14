#pragma once

// ! Deprecated
// TODO: remove

#include <functional>
#include <string_view>

#include "pipeline/action_interface.hpp"

namespace dtnproxy::pipeline {

struct RclMsgHash {
    // !Only properly works with timestamped msgs!
    std::size_t operator()(const rcl_serialized_message_t& msg) const noexcept {
        const char* data = reinterpret_cast<const char*>(msg.buffer);
        return std::hash<std::string_view>{}(std::string_view(data, msg.buffer_length));
    }
};

class LoggingAction : public IAction {
private:
    const uint SEQUENCE_NR = 0;
    const Direction dir = Direction::INOUT;

public:
    Direction direction() override;
    uint order() override;
    bool run(PipelineMessage& pMsg) override;
};

}  // namespace dtnproxy::pipeline
