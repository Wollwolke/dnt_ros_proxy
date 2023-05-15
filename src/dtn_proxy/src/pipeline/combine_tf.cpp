#include "pipeline/combine_tf.hpp"

#include <netinet/in.h>
#include <tf2_ros/buffer.h>

#include <cstdint>
#include <cstring>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/serialized_message.hpp>

namespace dtnproxy::pipeline {

void CombineTfAction::appendMessage(std::vector<uint8_t>& buffer, rclcpp::SerializedMessage& msg,
                                    bool empty) {
    uint32_t msgLength = 0;
    if (!empty) {
        msgLength = htonl(static_cast<uint32_t>(msg.get_rcl_serialized_message().buffer_length));
    }

    // append serialized msg length
    auto* msgLengthPtr = reinterpret_cast<uint8_t*>(&msgLength);
    buffer.insert(buffer.end(), msgLengthPtr, msgLengthPtr + sizeof(msgLength));

    if (!empty) {
        // append msg
        auto cdrMsg = msg.get_rcl_serialized_message();
        buffer.insert(buffer.end(), cdrMsg.buffer, cdrMsg.buffer + cdrMsg.buffer_length);
    }
}

CombineTfAction::CombineTfAction(std::vector<std::string> params, rclcpp::Node& nodeHandle) {
    if (params.size() != 2) {
        std::cout << "CombineTF: Exactly two parameters required - sourceFrameId, targetFrameId"
                  << std::endl;
    } else {
        sourceFrame = params[0];
        targetFrame = params[1];
    }

    tfBuffer = std::make_unique<tf2_ros::Buffer>(nodeHandle.get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
}

Direction CombineTfAction::direction() { return dir; }

uint CombineTfAction::order() { return SEQUENCE_NR; }

bool CombineTfAction::run(PipelineMessage& pMsg) {
    std::vector<uint8_t> buffer;
    rclcpp::SerializedMessage serializedTf;

    // Append TF Message
    if (sourceFrame.empty() || targetFrame.empty()) {
        std::cout << "CombineTF: ⚠ No Transform added - Check sourceFrame / targetFrame parameter"
                  << std::endl;
        appendMessage(buffer, serializedTf, true);
    } else {
        try {
            // TODO: use header timestamp, if available
            geometry_msgs::msg::TransformStamped transform =
                tfBuffer->lookupTransform(targetFrame, sourceFrame, tf2::TimePointZero);
            tfSerialization.serialize_message(&transform, &serializedTf);
            appendMessage(buffer, serializedTf);

        } catch (const tf2::TransformException& ex) {
            std::cout << "CombineTF: ⚠ No Transform added - Could not transform '" << sourceFrame
                      << "' to '" << targetFrame << "': " << ex.what() << std::endl;
            appendMessage(buffer, serializedTf, true);
        }
    }

    // Append Topic Message
    appendMessage(buffer, *pMsg.serializedMessage);

    pMsg.serializedMessage->reserve(buffer.size());
    auto* cdrMsg = &pMsg.serializedMessage->get_rcl_serialized_message();
    std::memcpy(cdrMsg->buffer, &buffer.front(), buffer.size());
    cdrMsg->buffer_length = buffer.size();

    return true;
}

}  // namespace dtnproxy::pipeline
