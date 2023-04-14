#pragma once

#include <memory>
#include <rclcpp/serialized_message.hpp>
#include <string_view>
#include <vector>

#include "configuration.hpp"
#include "dtnd_client.hpp"
#include "stats_recorder.hpp"

namespace dtnproxy::ros {

class ManagerBase {
protected:
    struct CdrMsgHash {
        // !Only properly works with timestamped msgs!
        std::size_t operator()(const rcl_serialized_message_t& msg) const noexcept {
            const char* data = reinterpret_cast<const char*>(msg.buffer);
            return std::hash<std::string_view>{}(std::string_view(data, msg.buffer_length));
        }
    };

    // TODO: validate this offset!!
    // Stats is using no overhead for DTN msgs, so ROS Overhead is also set to zero
    // const uint32_t CDR_MSG_SIZE_OFFSET = sizeof(size_t) + sizeof(size_t);
    const uint32_t CDR_MSG_SIZE_OFFSET = 0;

    rclcpp::Node& nodeHandle;
    const std::unique_ptr<Logger>& log;
    std::shared_ptr<StatsRecorder> stats;
    std::shared_ptr<DtndClient> dtn;
    conf::RosConfig config;

    ManagerBase(rclcpp::Node& nodeHandle, conf::RosConfig config, std::shared_ptr<DtndClient> dtn,
                const std::unique_ptr<Logger>& log);

    uint32_t getRosMsgSize(const std::shared_ptr<rclcpp::SerializedMessage>& msg) const;

    uint32_t buildDtnPayload(std::vector<uint8_t>& payload,
                             const std::shared_ptr<rclcpp::SerializedMessage>& msg, int reqId = -1);
    std::string prefixTopic(const std::string& topicName, bool isService);

public:
    void setStatsRecorder(std::shared_ptr<StatsRecorder> statsRecorder);
};

}  // namespace dtnproxy::ros
