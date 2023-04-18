#include "pipeline/split_topics.hpp"

#include <netinet/in.h>
#include <rcl/types.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace dtnproxy::pipeline {

SplitTopicsAction::SplitTopicsAction(std::string currentTopic, std::vector<std::string> topics,
                                     injectMsgCb_t injectMsgCb)
    : topicsToSplit(std::move(topics)),
      currentTopic(std::move(currentTopic)),
      injectMsgCb(std::move(injectMsgCb)) {}

Direction SplitTopicsAction::direction() { return dir; }

uint SplitTopicsAction::order() { return SEQUENCE_NR; }

bool SplitTopicsAction::run(PipelineMessage& pMsg) {
    auto cdrMsg = pMsg.serializedMessage->get_rcl_serialized_message();

    uint32_t msgLength;
    const auto SIZE_OF_LEN = sizeof(msgLength);
    size_t offset = 0;
    std::vector<uint8_t> buffer;

    for (const auto& topic : topicsToSplit) {
        std::memcpy(&msgLength, cdrMsg.buffer + offset, SIZE_OF_LEN);
        offset += SIZE_OF_LEN;
        msgLength = ntohl(msgLength);

        if (0 != msgLength) {
            buffer.assign(cdrMsg.buffer + offset, cdrMsg.buffer + offset + msgLength);
            offset += msgLength;

            rcl_serialized_message_t newMsg{
                &buffer.front(),                     // buffer
                static_cast<size_t>(buffer.size()),  // buffer_length
                static_cast<size_t>(buffer.size()),  // buffer_capacity
                rcl_get_default_allocator()          // allocator
            };

            if (currentTopic != topic) {
                auto msgPtr = std::make_shared<rclcpp::SerializedMessage>(newMsg);
                injectMsgCb(topic, std::move(msgPtr));
            } else {
                *pMsg.serializedMessage = newMsg;
            }
        }
    }

    return true;
}

}  // namespace dtnproxy::pipeline
