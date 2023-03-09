#pragma once

#include <chrono>

#include "pipeline/action_interface.hpp"

namespace dtnproxy::pipeline {

class RateLimitAction : public IAction {
private:
    const int MS_IN_SECOND = 1000;
    const unsigned int deltaT;
    std::chrono::time_point<std::chrono::steady_clock> lastMsgSentTime;

public:
    RateLimitAction(unsigned int secondsBetweenMsgs);

    bool run(std::shared_ptr<rclcpp::SerializedMessage> msg) override;
};

}  // namespace dtnproxy::pipeline
