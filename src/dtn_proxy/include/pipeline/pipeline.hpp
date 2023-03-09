#pragma once

#include <memory>
#include <rclcpp/serialized_message.hpp>
#include <vector>

#include "pipeline/action_interface.hpp"

namespace dtnproxy::pipeline {

class Pipeline {
private:
    std::vector<std::unique_ptr<IAction>> actions;

public:
    void appendActions(std::unique_ptr<IAction> actionPtr);
    bool run(std::shared_ptr<rclcpp::SerializedMessage> msg);
};

}  // namespace dtnproxy::pipeline
