#include "ros/topic_manager.hpp"

#include <arpa/inet.h>

#include <memory>
#include <rclcpp/serialized_message.hpp>
#include <string>
#include <utility>
#include <vector>

#include "pipeline/pipeline.hpp"
#include "ros/dtn_msg_type.hpp"

namespace dtnproxy::ros {

void TopicManager::topicCallback(const std::string& topic, const std::string& type,
                                 std::shared_ptr<rclcpp::SerializedMessage> msg) {
    auto rosMsgSize = getRosMsgSize(msg);
    if (stats) stats->rosReceived(topic, type, rosMsgSize, DtnMsgType::TOPIC);

    // run optimization pipeline before sending msg over DTN
    if (subscriber.at(topic).second.run(msg)) {
        std::vector<uint8_t> payload;
        buildDtnPayload(payload, msg);

        dtn->sendMessage(payload, topic, DtnMsgType::TOPIC);
        if (stats) stats->dtnSent(topic, type, payload.size(), DtnMsgType::TOPIC);
    }
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

void TopicManager::onDtnMessage(const std::string& topic, std::vector<uint8_t>& data) {
    rcl_serialized_message_t cdrMsg{
        &data.front(),               // buffer
        data.size(),                 // buffer_length
        data.size(),                 // buffer_capacity
        rcl_get_default_allocator()  // allocator
    };
    auto msg = std::make_shared<rclcpp::SerializedMessage>(cdrMsg);

    // run optimization pipeline before sending ROS msgs
    if (publisher.at(topic).second.run(msg)) {
        publisher.at(topic).first->publish(*msg);

        // TODO: find msgType in rosConfig
        if (stats) {
            stats->rosSent(topic, "unknown", getRosMsgSize(msg), DtnMsgType::TOPIC);
        }
    }
}

void TopicManager::initPublisher() {
    auto qos = rclcpp::QoS(10);

    for (const auto& [topic, type, profile] : config.pubTopics) {
        pipeline::Pipeline pipeline(pipeline::Direction::OUT, type);
        pipeline.initPipeline(config.profiles, profile);

        auto pubTopic = prefixTopic(topic, false);
        publisher.insert_or_assign(
            topic, std::make_pair(nodeHandle.create_generic_publisher(pubTopic, type, qos),
                                  std::move(pipeline)));

        log->INFO() << "Providing topic:\t" << pubTopic;
    }
}

void TopicManager::initSubscriber() {
    auto qos = rclcpp::QoS(10);

    for (const auto& [topic, type, profile] : config.subTopics) {
        pipeline::Pipeline pipeline(pipeline::Direction::IN, type);
        pipeline.initPipeline(config.profiles, profile);

        auto cb = std::bind(&TopicManager::topicCallback, this, topic, type, std::placeholders::_1);

        subscriber.insert_or_assign(
            topic, std::make_pair(nodeHandle.create_generic_subscription(topic, type, qos, cb),
                                  std::move(pipeline)));

        log->INFO() << "Subscribed to topic:\t" << topic;
    }
}

}  // namespace dtnproxy::ros
