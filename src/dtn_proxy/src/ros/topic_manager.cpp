#include "ros/topic_manager.hpp"

#include <arpa/inet.h>

#include <memory>
#include <rclcpp/serialized_message.hpp>
#include <string>
#include <vector>

#include "ros/dtn_msg_type.hpp"

namespace dtnproxy::ros {

void TopicManager::topicCallback(const std::string& topic, const std::string& type,
                                 std::shared_ptr<rclcpp::SerializedMessage> msg) {
    std::vector<uint8_t> payload;
    auto rosMsgSize = buildDtnPayload(payload, msg);
    if (stats) stats->rosReceived(topic, type, rosMsgSize, DtnMsgType::TOPIC);

    dtn->sendMessage(payload, topic, DtnMsgType::TOPIC);
    if (stats) stats->dtnSent(topic, type, payload.size(), DtnMsgType::TOPIC);
}

// TODO: Use this dynamic type support stuff to get the header...
// void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) {
//     using MessageT = geometry_msgs::msg::PoseStamped;
//     MessageT string_msg;

// auto msg_type = "geometry_msgs/msg/PoseStamped";
// auto lib = rosbag2_cpp::get_typesupport_library(msg_type, "rosidl_typesupport_cpp");
// auto typesupport =
//     rosbag2_cpp::get_typesupport_handle(msg_type, "rosidl_typesupport_cpp", lib);

//     auto cap = msg->capacity();
//     auto size = msg->size();
//     auto test = msg->get_rcl_serialized_message();
// auto serializer = rclcpp::SerializationBase(typesupport);
//     serializer.deserialize_message(msg.get(), &string_msg);
//     auto header_timestamp =
//         message_filters::message_traits::HasHeader<MessageT>::value;
//     RCLCPP_INFO(this->get_logger(), "I heard: %s",
//     string_msg.data.c_str());
// }

// void test(std::string type) {
//     auto lib = rosbag2_cpp::get_typesupport_library(type, "rosidl_typesupport_cpp");
//     auto typesupport = rosbag2_cpp::get_typesupport_handle(type,
//     "rosidl_typesupport_cpp", lib);
// }

TopicManager::TopicManager(rclcpp::Node& nodeHandle, conf::RosConfig config,
                           std::shared_ptr<DtndClient> dtn, const std::unique_ptr<Logger>& log)
    : ManagerBase(nodeHandle, config, dtn, log) {}

void TopicManager::onDtnMessage(const std::string& topic, std::vector<uint8_t>& data,
                                uint32_t size) {
    rcl_serialized_message_t cdrMsg{
        &data.front(),               // buffer
        size,                        // buffer_length
        size,                        // buffer_capacity
        rcl_get_default_allocator()  // allocator
    };

    // TODO: find msgType in rosConfig
    publisher.at(topic)->publish(rclcpp::SerializedMessage(cdrMsg));
    if (stats) {
        stats->rosSent(topic, "unknown",
                       static_cast<uint32_t>(cdrMsg.buffer_length) + CDR_MSG_SIZE_OFFSET,
                       DtnMsgType::TOPIC);
    }
}

void TopicManager::initPublisher() {
    auto qos = rclcpp::QoS(10);

    for (const auto& [topic, type] : config.pubTopics) {
        auto pubTopic = prefixTopic(topic, false);
        publisher.insert_or_assign(topic, nodeHandle.create_generic_publisher(pubTopic, type, qos));

        log->INFO() << "Providing topic:\t" << pubTopic;
    }
}

void TopicManager::initSubscriber() {
    auto qos = rclcpp::QoS(10);

    for (const auto& [topic, type] : config.subTopics) {
        auto cb = std::bind(&TopicManager::topicCallback, this, topic, type, std::placeholders::_1);
        subscriber.insert_or_assign(topic,
                                    nodeHandle.create_generic_subscription(topic, type, qos, cb));

        log->INFO() << "Subscribed to topic:\t" << topic;
    }
}

}  // namespace dtnproxy::ros