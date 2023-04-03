#pragma once

#include <chrono>

#include "pipeline/action_interface.hpp"

namespace dtnproxy::pipeline {

class RateLimitAction : public IAction {
private:
    const uint SEQUENCE_NR = 1;
    const int MS_IN_SECOND = 1000;
    const unsigned int deltaT;
    const Direction dir = Direction::IN;

    std::chrono::time_point<std::chrono::steady_clock> lastMsgSentTime;

public:
    RateLimitAction(unsigned int secondsBetweenMsgs);

    Direction direction() override;
    uint order() override;
    bool run(PipelineMessage& pMsg) override;
};

}  // namespace dtnproxy::pipeline
