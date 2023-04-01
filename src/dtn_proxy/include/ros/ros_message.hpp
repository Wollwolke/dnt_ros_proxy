#pragma once

#include <memory>
#include <rclcpp/serialized_message.hpp>
#include <ros2_babel_fish/babel_fish.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
// #include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <utility>

namespace dtnproxy::ros {

class RosMessage {
    using handles_t = std::pair<const rosidl_message_type_support_t*,
                                const rosidl_typesupport_introspection_cpp::MessageMembers*>;

private:
    handles_t handles;
    void initTsHanldes(const std::string& msgType);

public:
    ros2_babel_fish::MessageTypeSupport::ConstSharedPtr msgTypeSupport;
    std::shared_ptr<void> data;
    rclcpp::SerializedMessage serializedMsg;

    // TODO: cleanup ⬇
    RosMessage(std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string& msgType);
    RosMessage(std::shared_ptr<void> msg, const std::string& msgType);
    ~RosMessage();
    RosMessage(const RosMessage&) = delete;
    RosMessage& operator=(const RosMessage&) = delete;
};

}  // namespace dtnproxy::ros
