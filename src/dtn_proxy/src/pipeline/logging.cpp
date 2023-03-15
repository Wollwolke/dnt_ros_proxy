#include "pipeline/logging.hpp"

#include <chrono>
#include <iostream>

namespace dtnproxy::pipeline {

Direction LoggingAction::direction() { return dir; }

uint LoggingAction::order() { return SEQUENCE_NR; }

bool LoggingAction::run(std::shared_ptr<rclcpp::SerializedMessage> msg) {
    auto timeNow = std::chrono::system_clock::now();
    auto hash = RclMsgHash{}(msg->get_rcl_serialized_message());
    std::cout << "[HASH]" << hash << "[TIME]" << timeNow.time_since_epoch().count() << std::endl;
    return true;
}

}  // namespace dtnproxy::pipeline
