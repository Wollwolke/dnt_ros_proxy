#include "pipeline/split_tf.hpp"

#include <netinet/in.h>
#include <rcl/types.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <memory>
#include <vector>

namespace dtnproxy::pipeline {

SplitTfAction::SplitTfAction(rclcpp::Node& nodeHandle) {
    tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(nodeHandle);
}

Direction SplitTfAction::direction() { return dir; }

uint SplitTfAction::order() { return SEQUENCE_NR; }

bool SplitTfAction::run(PipelineMessage& pMsg) {
    auto cdrMsg = pMsg.serializedMessage->get_rcl_serialized_message();

    uint32_t msgLength;
    const auto SIZE_OF_LEN = sizeof(msgLength);
    size_t offset = 0;
    std::vector<uint8_t> buffer;

    for (size_t i = 0; i < 2; ++i) {
        std::memcpy(&msgLength, cdrMsg.buffer + offset, SIZE_OF_LEN);
        offset += SIZE_OF_LEN;
        msgLength = ntohl(msgLength);

        if (0 != msgLength) {
            // send TF Message
            buffer.assign(cdrMsg.buffer + offset, cdrMsg.buffer + offset + msgLength);
            offset += msgLength;

            if (i == 0) {
                // send TF Message
                rclcpp::SerializedMessage serializedTf(buffer.size());
                auto* newMsg = &serializedTf.get_rcl_serialized_message();
                std::memcpy(newMsg->buffer, &buffer.front(), buffer.size());
                newMsg->buffer_length = buffer.size();
                geometry_msgs::msg::TransformStamped transform;
                tfSerialization.deserialize_message(&serializedTf, &transform);
                tfBroadcaster->sendTransform(transform);
            } else {
                // forward Topic Message
                pMsg.serializedMessage->reserve(buffer.size());
                auto* newCdrMsg = &pMsg.serializedMessage->get_rcl_serialized_message();
                newCdrMsg->buffer_length = buffer.size();
                std::memcpy(newCdrMsg->buffer, &buffer.front(), buffer.size());
            }
        }
    }

    return true;
}

}  // namespace dtnproxy::pipeline
