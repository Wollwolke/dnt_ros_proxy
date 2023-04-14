#include "ros/transfer.hpp"

#include <arpa/inet.h>

#include <memory>
#include <vector>

namespace dtnproxy::ros {

Transfer::Transfer(rclcpp::Node& nodeHandle, conf::RosConfig config,
                   std::shared_ptr<DtndClient> dtn)
    : topics(nodeHandle, config, dtn, log), services(nodeHandle, config, dtn, log) {
    log = std::make_unique<Logger>("ros");
}

std::pair<std::string, ros::DtnMsgType> Transfer::splitEndpointAndType(
    const std::string& typedEndpoint) {
    size_t stringPos;

    // Topic
    stringPos = typedEndpoint.find("rt_");
    if (std::string::npos != stringPos) {
        return std::make_pair(typedEndpoint.substr(stringPos + 3), DtnMsgType::TOPIC);
    }

    // Response
    stringPos = typedEndpoint.find("rr_");
    if (std::string::npos != stringPos) {
        return std::make_pair(typedEndpoint.substr(stringPos + 3), DtnMsgType::RESPONSE);
    }

    // Request
    stringPos = typedEndpoint.find("rq_");
    if (std::string::npos != stringPos) {
        return std::make_pair(typedEndpoint.substr(stringPos + 3), DtnMsgType::REQUEST);
    }

    return std::make_pair("", DtnMsgType::INVALID);
}

// TODO: naming
void Transfer::initServers() {
    static bool done = false;
    if (!done) {
        topics.initPublisher();
        services.initClients();
        done = true;
    }
}
// TODO: naming
void Transfer::initClients() {
    static bool done = false;
    if (!done) {
        topics.initSubscriber();
        services.initServers();
        done = true;
    }
}

void Transfer::enableStatsRecorder(std::shared_ptr<StatsRecorder> statsRecorder) {
    stats = statsRecorder;
    topics.setStatsRecorder(statsRecorder);
    services.setStatsRecorder(statsRecorder);
}

void Transfer::onDtnMessage(const data::WsReceive& bundle) {
    constexpr auto SIZE_OF_HEADER_ID = 1;

    auto data = bundle.data;
    auto typedEndpoint = bundle.dst;

    auto [topic, type] = splitEndpointAndType(typedEndpoint);
    // TODO: get msg type for stats
    if (stats) stats->dtnReceived(topic, "unknown", data.size(), type);

    switch (type) {
        case DtnMsgType::TOPIC: {
            topics.onDtnMessage(topic, data);
        } break;
        case DtnMsgType::REQUEST: {
            std::vector<uint8_t> buffer(data.begin() + SIZE_OF_HEADER_ID, data.end());
            uint8_t headerId;
            memcpy(&headerId, &data.front(), SIZE_OF_HEADER_ID);
            services.onDtnRequest(topic, buffer, headerId);
        } break;
        case DtnMsgType::RESPONSE: {
            std::vector<uint8_t> buffer(data.begin() + SIZE_OF_HEADER_ID, data.end());
            uint8_t headerId;
            memcpy(&headerId, &data.front(), SIZE_OF_HEADER_ID);
            services.onDtnResponse(topic, buffer, headerId);
        } break;
        case DtnMsgType::INVALID:
        default:
            // Not a message for us, ignoring...
            return;
    }
}

}  // namespace dtnproxy::ros
