#include "pipeline/rate_limit.hpp"

#include <chrono>

#include "pipeline/action_interface.hpp"

namespace dtnproxy::pipeline {

RateLimitAction::RateLimitAction(unsigned int secondsBetweenMsgs)
    : deltaT(secondsBetweenMsgs * MS_IN_SECOND) {}

Direction RateLimitAction::direction() { return dir; }

uint RateLimitAction::order(const Direction& /*pipelineDir*/) { return SEQUENCE_NR_IN; }

bool RateLimitAction::run(PipelineMessage& /*pMsg*/, const Direction& /*pipelineDir*/) {
    using namespace std::chrono;

    auto timeNow = steady_clock::now();
    if (duration_cast<milliseconds>(timeNow - lastMsgSentTime).count() > deltaT) {
        lastMsgSentTime = timeNow;
        return true;
    }
    return false;
}

}  // namespace dtnproxy::pipeline
