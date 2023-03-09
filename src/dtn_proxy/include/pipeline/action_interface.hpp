#pragma once

#include <memory>
#include <rclcpp/serialized_message.hpp>

namespace dtnproxy::pipeline {

class IAction {
private:
public:
    virtual bool run(std::shared_ptr<rclcpp::SerializedMessage> msg) = 0;
};

}  // namespace dtnproxy::pipeline
