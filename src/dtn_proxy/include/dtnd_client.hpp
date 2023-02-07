#pragma once
#include <httplib.h>

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "configuration.hpp"
#include "logger.hpp"
#include "ws_client.hpp"
#include "ws_datatypes.hpp"

class DtndClient {
private:
    using Result = struct Result {
        bool success;
        std::string content;
        Result(bool success, std::string content);
    };
    using messageHandler_t = std::function<void(const data::WsReceive&)>;

    enum WsState { NOTSET = -1, ERROR, CONNECTED };

    const proxyConfig::DtnConfig config;

    std::unique_ptr<httplib::Client> http;
    std::unique_ptr<WsClient> ws;
    std::unique_ptr<Logger> log;

    WsState wsConnected = WsState::NOTSET;
    std::mutex wsMutex;
    std::condition_variable wsCV;

    messageHandler_t messageHandler;
    std::string localNodeId;
    std::vector<std::string> endpointsToRegister;

    Result getRequest(std::string path);
    bool getLocalNodeId();
    bool registerSubscribeEndpoints();

public:
    DtndClient(const proxyConfig::DtnConfig& config, std::string loggerName = "dtn_lib");

    void setMessageHandler(messageHandler_t h);

    void onConnectionStatus(const bool success);
    void onBundle(const std::string& bundle);

    bool connect(const std::vector<std::string>& endpoints);
    void sendMessage(const std::vector<uint8_t>& payload, const std::string& endpoint);
};
