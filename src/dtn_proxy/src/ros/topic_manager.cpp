#include "ros/topic_manager.hpp"

#include <arpa/inet.h>

#include <cstdint>
#include <memory>
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
    if (stats) {
        auto hash = CdrMsgHash{}(msg->get_rcl_serialized_message(), topic);
        stats->rosReceived(topic, type, rosMsgSize, DtnMsgType::TOPIC, hash);
    }

    pipeline::PipelineMessage pMsg{std::move(msg)};
    // run optimization pipeline before sending msg over DTN
    if (subscriber.at(topic).second.run(pMsg)) {
        std::vector<uint8_t> payload;
        buildDtnPayload(payload, pMsg.serializedMessage);

        auto bundleFlags = pMsg.markExpired ? DtndClient::BundleFlags::BUNDLE_REMOVE_OLDER_BUNDLES
                                            : DtndClient::BundleFlags::NO_FLAGS;

        DtndClient::Message dtnMsg{
            std::move(payload),       // std::vector<uint8_t> payload,
            config.nodePrefix + topic,  // std::string endpoint,
            DtnMsgType::TOPIC,          // ros::DtnMsgType msgType,
            bundleFlags,                // uint64_t bundleFlags = 0,
            pMsg.lifetime,              // uint64_t lifetime = 0,
        };

        dtn->sendMessage(dtnMsg);
        if (stats) stats->dtnSent(topic, type, dtnMsg.payload.size(), DtnMsgType::TOPIC);
    }
}

void TopicManager::dtnMsgCallback(const std::string& topic,
                                  std::shared_ptr<rclcpp::SerializedMessage> msg,
                                  bool skipPipeline) {
    pipeline::PipelineMessage pMsg{std::move(msg)};
    // run optimization pipeline before sending ROS msgs
    if (skipPipeline || publisher.at(topic).second.run(pMsg)) {
        publisher.at(topic).first->publish(*pMsg.serializedMessage);

        // TODO: find msgType in rosConfig
        if (stats) {
            auto hash = CdrMsgHash{}(pMsg.serializedMessage->get_rcl_serialized_message(), topic);
            stats->rosSent(topic, "unknown", getRosMsgSize(pMsg.serializedMessage),
                           DtnMsgType::TOPIC, hash);
        }
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
    dtnMsgCallback(topic, msg);
}

void TopicManager::initPublisher() {
    auto qos = rclcpp::QoS(10);

    // callback to inject msgs from other topics
    auto injectMsgCb = std::bind(&TopicManager::dtnMsgCallback, this, std::placeholders::_1,
                                 std::placeholders::_2, true);

    for (const auto& [topic, type, profile] : config.pubTopics) {
        pipeline::Pipeline pipeline(pipeline::Direction::OUT, type, topic);
        pipeline.initPipeline(config.profiles, profile, nodeHandle, injectMsgCb);

        auto pubTopic = prefixTopic(topic, false);
        publisher.insert_or_assign(
            topic, std::make_pair(nodeHandle.create_generic_publisher(pubTopic, type, qos),
                                  std::move(pipeline)));

        log->INFO() << "Providing topic:\t" << pubTopic;
    }
}

void TopicManager::initSubscriber() {
    auto qos = rclcpp::QoS(10);

    // msg store for combining msgs over multiple subscribers
    auto msgStore = std::make_shared<std::map<std::string, pipeline::PipelineMessage>>();

    for (const auto& [topic, type, profile] : config.subTopics) {
        pipeline::Pipeline pipeline(pipeline::Direction::IN, type, topic);
        pipeline.initPipeline(config.profiles, profile, nodeHandle, msgStore);

        auto cb = std::bind(&TopicManager::topicCallback, this, topic, type, std::placeholders::_1);

        subscriber.insert_or_assign(
            topic, std::make_pair(nodeHandle.create_generic_subscription(topic, type, qos, cb),
                                  std::move(pipeline)));

        log->INFO() << "Subscribed to topic:\t" << topic;
    }
}

}  // namespace dtnproxy::ros
