#pragma once

#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "logger.hpp"
#include "ros/manager_base.hpp"

namespace dtnproxy::ros {
class TopicManager : public ManagerBase {
private:
    std::map<std::string, std::shared_ptr<rclcpp::GenericSubscription>> subscriber;
    std::map<std::string, std::shared_ptr<rclcpp::GenericPublisher>> publisher;

    void topicCallback(const std::string& topic, const std::string& type,
                       std::shared_ptr<rclcpp::SerializedMessage> msg);

public:
    TopicManager();
    TopicManager(rclcpp::Node& nodeHandle, conf::RosConfig config, std::shared_ptr<DtndClient> dtn,
                 const std::unique_ptr<Logger>& log);

    void onDtnMessage(const std::string& topic, std::vector<uint8_t>& data, uint32_t size);
    void initPublisher();
    void initSubscriber();
};

}  // namespace dtnproxy::ros
