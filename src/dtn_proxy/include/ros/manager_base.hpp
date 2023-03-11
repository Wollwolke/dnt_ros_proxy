#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "configuration.hpp"
#include "dtnd_client.hpp"
#include "pipeline/pipeline.hpp"
#include "stats_recorder.hpp"

namespace dtnproxy::ros {

class ManagerBase {
protected:
    // TODO: validate this offset!!
    const uint32_t CDR_MSG_SIZE_OFFSET = sizeof(size_t) + sizeof(size_t);

    rclcpp::Node& nodeHandle;
    const std::unique_ptr<Logger>& log;
    std::shared_ptr<StatsRecorder> stats;
    std::shared_ptr<DtndClient> dtn;
    conf::RosConfig config;

    ManagerBase(rclcpp::Node& nodeHandle, conf::RosConfig config, std::shared_ptr<DtndClient> dtn,
                const std::unique_ptr<Logger>& log);

    uint32_t buildDtnPayload(std::vector<uint8_t>& payload,
                             const std::shared_ptr<rclcpp::SerializedMessage>& msg, int reqId = -1);
    std::string prefixTopic(const std::string& topicName, bool isService);

public:
    void setStatsRecorder(std::shared_ptr<StatsRecorder> statsRecorder);
};

}  // namespace dtnproxy::ros
