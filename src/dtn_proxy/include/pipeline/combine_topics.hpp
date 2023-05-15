#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "pipeline/action_interface.hpp"
#include "pipeline/pipeline_msg.hpp"

namespace dtnproxy::pipeline {

class CombineTopicsAction : public IAction {
private:
    using msgStorePtr_t = std::shared_ptr<std::map<std::string, PipelineMessage>>;

    // TODO: figure out the correct sequence for all modules
    const uint SEQUENCE_NR = 99;
    const Direction dir = Direction::IN;

    const std::vector<std::string> topicsToCombine;
    const std::string currentTopic;
    msgStorePtr_t msgStore;

    static void appendMessage(std::vector<uint8_t>& buffer,
                              std::shared_ptr<rclcpp::SerializedMessage> msg);

public:
    CombineTopicsAction(std::string currentTopic, std::vector<std::string> topics,
                        msgStorePtr_t msgStore);

    Direction direction() override;
    uint order() override;
    bool run(PipelineMessage& pMsg) override;
};

}  // namespace dtnproxy::pipeline
