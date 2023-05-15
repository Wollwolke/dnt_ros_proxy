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

            auto newSerializedMessage = std::make_shared<rclcpp::SerializedMessage>(buffer.size());
            auto* newMsg = &newSerializedMessage->get_rcl_serialized_message();
            std::memcpy(newMsg->buffer, &buffer.front(), buffer.size());
            newMsg->buffer_length = buffer.size();

            if (currentTopic != topic) {
                injectMsgCb(topic, std::move(newSerializedMessage));
            } else {
                pMsg.serializedMessage.swap(newSerializedMessage);
            }
        }
    }

    return true;
}

}  // namespace dtnproxy::pipeline
