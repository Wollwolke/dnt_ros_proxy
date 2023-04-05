#include "pipeline/combine_topics.hpp"

#include <netinet/in.h>

#include <cstddef>
#include <cstdint>
#include <utility>

namespace dtnproxy::pipeline {

void CombineTopicsAction::appendMessage(std::vector<uint8_t>& buffer,
                                        std::shared_ptr<rclcpp::SerializedMessage> msg) {
    uint32_t msgLength = 0;

    if (nullptr != msg) {
        msgLength = htonl(static_cast<uint32_t>(msg->get_rcl_serialized_message().buffer_length));
    }

    // append serialized msg length
    auto* msgLengthPtr = reinterpret_cast<uint8_t*>(&msgLength);
    buffer.insert(buffer.end(), msgLengthPtr, msgLengthPtr + sizeof(msgLength));

    // append msg
    if (nullptr != msg) {
        auto cdrMsg = msg->get_rcl_serialized_message();
        buffer.insert(buffer.end(), cdrMsg.buffer, cdrMsg.buffer + cdrMsg.buffer_length);
    }
}

CombineTopicsAction::CombineTopicsAction(std::string currentTopic, std::vector<std::string> topics,
                                         msgStorePtr_t msgStore)
    : topicsToCombine(std::move(topics)),
      currentTopic(std::move(currentTopic)),
      msgStore(std::move(msgStore)) {}

Direction CombineTopicsAction::direction() { return dir; }

uint CombineTopicsAction::order() { return SEQUENCE_NR; }

bool CombineTopicsAction::run(PipelineMessage& pMsg) {
    // running on not the main topic
    if (currentTopic != topicsToCombine[0]) {
        msgStore->insert_or_assign(currentTopic, pMsg);
        return false;
    }

    // running on the main topic
    // append all msgs to buffer
    std::vector<uint8_t> buffer;
    appendMessage(buffer, pMsg.serializedMessage);

    for (const auto& topic : topicsToCombine) {
        if (currentTopic == topic) continue;

        auto msgIt = msgStore->find(topic);
        if (msgIt == msgStore->end()) {
            appendMessage(buffer, nullptr);
        } else {
            appendMessage(buffer, msgIt->second.serializedMessage);
        }
    }
    msgStore->clear();

    rcl_serialized_message_t newMsg{
        &buffer.front(),                     // buffer
        static_cast<size_t>(buffer.size()),  // buffer_length
        static_cast<size_t>(buffer.size()),  // buffer_capacity
        rcl_get_default_allocator()          // allocator
    };

    *pMsg.serializedMessage = newMsg;
    return true;
}

}  // namespace dtnproxy::pipeline
