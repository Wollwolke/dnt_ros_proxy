#pragma once

#include <memory>
#include <ros2_babel_fish/babel_fish.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <string>
#include <vector>

#include "pipeline/action_interface.hpp"

namespace dtnproxy::pipeline {

class OnChangeAction : public IAction {
private:
    const uint SEQUENCE_NR = 2;
    const Direction dir = Direction::IN;

    std::vector<std::shared_ptr<ros2_babel_fish::Message>> oldMsg;

    ros2_babel_fish::MessageTypeSupport::ConstSharedPtr msgTypeSupport;
    std::unique_ptr<ros2_babel_fish::MessageMembersIntrospection> msgMemberIntro;
    const rosidl_typesupport_introspection_cpp::MessageMembers* msgMembers;

public:
    OnChangeAction(std::string msgType);

    Direction direction() override;
    uint order() override;
    bool run(PipelineMessage& pMsg) override;
};

}  // namespace dtnproxy::pipeline
