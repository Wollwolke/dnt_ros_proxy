#include "ros/ros_message.hpp"

#include <cstdlib>
#include <memory>
#include <rclcpp/serialization.hpp>
#include <utility>

namespace dtnproxy::ros {

void RosMessage::initTsHanldes(const std::string& msgType) {
    msgTypeSupport = ros2_babel_fish::BabelFish().get_message_type_support(msgType);
    const auto* tsHandle = &msgTypeSupport->type_support_handle;
    const auto* introTsHandle =
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(
            msgTypeSupport->introspection_type_support_handle.data);

    handles = std::make_pair(tsHandle, introTsHandle);
}

RosMessage::RosMessage(std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string& msgType) {
    initTsHanldes(msgType);

    data = std::shared_ptr<void>(malloc(handles.second->size_of_), free);
    handles.second->init_function(data.get(), rosidl_runtime_cpp::MessageInitialization::ALL);
    rclcpp::SerializationBase(handles.first).deserialize_message(msg.get(), data.get());
}

RosMessage::RosMessage(std::shared_ptr<void> msg, const std::string& msgType) {
    initTsHanldes(msgType);

    // data = std::shared_ptr<void>(malloc(handles.second->size_of_), free);
    // handles.second->init_function(data.get(), rosidl_runtime_cpp::MessageInitialization::ALL);
    rclcpp::SerializationBase(handles.first).serialize_message(msg.get(), &serializedMsg);
}

RosMessage::~RosMessage() { handles.second->fini_function(data.get()); }

}  // namespace dtnproxy::ros
