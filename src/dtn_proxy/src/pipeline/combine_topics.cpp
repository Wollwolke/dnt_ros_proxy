#include "pipeline/combine_topics.hpp"

#include <netinet/in.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <utility>

namespace dtnproxy::pipeline {

void CombineTopicsAction::appendMessage(std::vector<uint8_t>& buffer,
                                        std::shared_ptr<rclcpp::SerializedMessage> msg) {
    uint32_t msgLength =
        htonl(static_cast<uint32_t>(msg->get_rcl_serialized_message().buffer_length));

    // append serialized msg length
    auto* msgLengthPtr = reinterpret_cast<uint8_t*>(&msgLength);
    buffer.insert(buffer.end(), msgLengthPtr, msgLengthPtr + sizeof(msgLength));

    // append msg
    auto cdrMsg = msg->get_rcl_serialized_message();
    buffer.insert(buffer.end(), cdrMsg.buffer, cdrMsg.buffer + cdrMsg.buffer_length);
}

CombineTopicsAction::CombineTopicsAction(std::string currentTopic, std::vector<std::string> topics,
                                         msgStorePtr_t msgStore)
    : topicsToCombine(std::move(topics)),
      currentTopic(std::move(currentTopic)),
      msgStore(std::move(msgStore)) {}

Direction CombineTopicsAction::direction() { return dir; }

uint CombineTopicsAction::order() { return SEQUENCE_NR; }

bool CombineTopicsAction::run(PipelineMessage& pMsg) {
    // check if all msgs exist in store
    bool allInStore = true;
    for (const auto& topic : topicsToCombine) {
        if (topic != currentTopic && msgStore->find(topic) == msgStore->end()) {
            allInStore = false;
        }
    }

    if (allInStore) {
        // all msgs in store -> ready to send
        std::vector<uint8_t> buffer;
        bool expireAllBundles = pMsg.markExpired;
        for (const auto& topic : topicsToCombine) {
            if (topic == currentTopic) {
                appendMessage(buffer, pMsg.serializedMessage);
            } else {
                auto msgIt = msgStore->find(topic);
                appendMessage(buffer, msgIt->second.serializedMessage);
                expireAllBundles &= msgIt->second.markExpired;
            }
        }
        msgStore->clear();

        pMsg.serializedMessage->reserve(buffer.size());
        auto* cdrMsg = &pMsg.serializedMessage->get_rcl_serialized_message();
        std::memcpy(cdrMsg->buffer, &buffer.front(), buffer.size());
        cdrMsg->buffer_length = buffer.size();

        pMsg.markExpired = expireAllBundles;

        return true;
    }

    // not all msgs in store yet -> add current msg to store
    msgStore->insert_or_assign(currentTopic, std::move(pMsg));
    return false;
}

}  // namespace dtnproxy::pipeline
