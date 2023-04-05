#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "pipeline/action_interface.hpp"
#include "pipeline/pipeline_msg.hpp"

namespace dtnproxy::pipeline {

class SplitTopicsAction : public IAction {
private:
    using injectMsgCb_t = std::function<void(const std::string& topic,
                                             std::shared_ptr<rclcpp::SerializedMessage> msg)>;

    // TODO: figure out the correct sequence for all modules
    const uint SEQUENCE_NR = 5;
    const Direction dir = Direction::OUT;

    const std::vector<std::string> topicsToSplit;
    const std::string currentTopic;
    injectMsgCb_t injectMsgCb;

public:
    SplitTopicsAction(std::string currentTopic, std::vector<std::string> topics,
                      injectMsgCb_t injectMsgCb);

    Direction direction() override;
    uint order() override;
    bool run(PipelineMessage& pMsg) override;
};

}  // namespace dtnproxy::pipeline
