#include "pipeline/on_change.hpp"

#include <cstddef>
#include <memory>
#include <rclcpp/serialization.hpp>
#include <string>
#include <utility>

#include "ros/type_helper.hpp"

namespace dtnproxy::pipeline {

OnChangeAction::OnChangeAction(std::string msgType) {
    using ros2_babel_fish::MessageMembersIntrospection;
    using rosidl_typesupport_introspection_cpp::MessageMembers;

    // init typesupport
    msgTypeSupport = ros2_babel_fish::BabelFish().get_message_type_support(msgType);
    msgMembers =
        static_cast<const MessageMembers *>(msgTypeSupport->introspection_type_support_handle.data);
    msgMemberIntro = std::make_unique<MessageMembersIntrospection>(
        MessageMembersIntrospection(msgMembers, msgTypeSupport->type_support_library));
}

Direction OnChangeAction::direction() { return dir; }

uint OnChangeAction::order() { return SEQUENCE_NR; }

bool OnChangeAction::run(PipelineMessage &pMsg) {
    auto rosMsg = ros::allocate_message(msgMembers);
    rclcpp::SerializationBase(&msgTypeSupport->type_support_handle)
        .deserialize_message(pMsg.serializedMessage.get(), rosMsg.get());
    auto fishMsg = ros2_babel_fish::CompoundMessage(*msgMemberIntro, rosMsg);

    // find header
    int headerIndex = -1;
    for (size_t i = 0; i < fishMsg.keys().size(); ++i) {
        if ("header" == fishMsg.keyAt(i)) {
            headerIndex = i;
            break;
        }
    }

    size_t cnt = 0;
    auto fishMsgParts = fishMsg.values();
    if (oldMsg.empty()) {
        for (auto msgPart : fishMsgParts) {
            if (headerIndex == cnt++) continue;
            oldMsg.push_back(std::move(msgPart));
        }
        return true;
    }

    auto equal = true;
    cnt = 0;
    for (size_t i = 0; i < fishMsgParts.size(); ++i) {
        if (headerIndex == i) continue;
        equal &= (*oldMsg[cnt] == *fishMsgParts[i]);
        oldMsg[cnt] = std::move(fishMsgParts[i]);
        cnt++;
    }

    return !equal;
}

}  // namespace dtnproxy::pipeline
