#pragma once

#include <memory>
#include <rclcpp/serialized_message.hpp>

namespace dtnproxy::pipeline {

enum Direction {
    IN = 1 << 0,
    OUT = 1 << 1,
    INOUT = 1 << 2,
};

class IAction {
private:
public:
    virtual bool run(std::shared_ptr<rclcpp::SerializedMessage> msg) = 0;
    virtual Direction direction() = 0;
};

}  // namespace dtnproxy::pipeline
