#pragma once
#include <httplib.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "configuration.hpp"
#include "logger.hpp"
#include "ros/dtn_msg_type.hpp"
#include "ws_client.hpp"
#include "ws_datatypes.hpp"

namespace dtnproxy {

class DtndClient {
private:
    using Result = struct Result {
        bool success;
        std::string content;
        Result(bool success = false, std::string content = "");
    };
    using messageHandler_t = std::function<void(const data::WsReceive&)>;
    using DtnEndpoint = std::pair<std::string, ros::DtnMsgType>;

    enum WsState { NOTSET = -1, ERROR, CONNECTED };

    const conf::DtnConfig config;

    std::unique_ptr<httplib::Client> http;
    std::unique_ptr<WsClient> ws;
    std::unique_ptr<Logger> log;

    WsState wsConnected = WsState::NOTSET;
    std::mutex wsMutex;
    std::condition_variable wsCV;

    messageHandler_t messageHandler;
    std::string localNodeId;
    std::vector<DtnEndpoint> endpointsToRegister;

    Result getRequest(std::string path);
    bool getLocalNodeId();
    bool registerSubscribeEndpoints();
    void buildEndpointId(std::string& endpoint, ros::DtnMsgType type);

public:
    using Message = struct Message {
        std::vector<uint8_t> payload;
        std::string endpoint;
        ros::DtnMsgType msgType;
        uint64_t lifetime = 0;
        bool expireOlderBundles = false;
    };

    explicit DtndClient(const conf::DtnConfig& config);

    void setMessageHandler(messageHandler_t h);

    void onConnectionStatus(bool success);
    void onBundle(const std::string& bundle);

    bool connect(const std::vector<DtnEndpoint>& endpoints);
    void sendMessage(const Message& dtnMsg);
};

}  // namespace dtnproxy
