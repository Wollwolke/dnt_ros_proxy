#include "pipeline/rate_limit.hpp"

#include <chrono>
#include <iostream>
#include <rclcpp/time.hpp>

namespace dtnproxy::pipeline {

RateLimitAction::RateLimitAction(unsigned int secondsBetweenMsgs)
    : deltaT(secondsBetweenMsgs * MS_IN_SECOND) {}

bool RateLimitAction::run(std::shared_ptr<rclcpp::SerializedMessage> /*msg*/) {
    using namespace std::chrono;

    auto timeNow = steady_clock::now();
    if (duration_cast<milliseconds>(timeNow - lastMsgSentTime).count() > deltaT) {
        lastMsgSentTime = timeNow;
        return true;
    }
    return false;
}

}  // namespace dtnproxy::pipeline
