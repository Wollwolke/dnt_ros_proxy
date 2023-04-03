#include "ros/topic_manager.hpp"

#include <arpa/inet.h>

#include <memory>
#include <rclcpp/serialized_message.hpp>
#include <string>
#include <utility>
#include <vector>

#include "dtnd_client.hpp"
#include "pipeline/pipeline.hpp"
#include "pipeline/pipeline_msg.hpp"
#include "ros/dtn_msg_type.hpp"

namespace dtnproxy::ros {

void TopicManager::topicCallback(const std::string& topic, const std::string& type,
                                 std::shared_ptr<rclcpp::SerializedMessage> msg) {
    auto rosMsgSize = getRosMsgSize(msg);
    if (stats) stats->rosReceived(topic, type, rosMsgSize, DtnMsgType::TOPIC);

    pipeline::PipelineMessage pMsg{std::move(msg)};
    // run optimization pipeline before sending msg over DTN
    if (subscriber.at(topic).second.run(pMsg)) {
        std::vector<uint8_t> payload;
        buildDtnPayload(payload, pMsg.serializedMessage);
        DtndClient::Message dtnMsg{std::move(payload), topic, DtnMsgType::TOPIC, pMsg.lifetime,
                                   pMsg.markExpired};

        dtn->sendMessage(dtnMsg);
        if (stats) stats->dtnSent(topic, type, dtnMsg.payload.size(), DtnMsgType::TOPIC);
    }
}

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

    pipeline::PipelineMessage pMsg{std::move(msg)};
    // run optimization pipeline before sending ROS msgs
    if (publisher.at(topic).second.run(pMsg)) {
        publisher.at(topic).first->publish(*pMsg.serializedMessage);

        // TODO: find msgType in rosConfig
        if (stats) {
            stats->rosSent(topic, "unknown", getRosMsgSize(pMsg.serializedMessage),
                           DtnMsgType::TOPIC);
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
