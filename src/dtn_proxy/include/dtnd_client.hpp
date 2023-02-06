#pragma once
#include <httplib.h>

#include <configuration.hpp>
#include <functional>
#include <logger.hpp>
#include <memory>
#include <vector>
#include <ws_client.hpp>

class DtndClient {
private:
    using Result = struct Result {
        bool success;
        std::string content;
        Result(bool success, std::string content);
    };
    using messageHandler_t = std::function<void(const std::vector<uint8_t>&)>;

    const proxyConfig::DtnConfig config;

    std::unique_ptr<httplib::Client> http;
    std::unique_ptr<WsClient> ws;
    std::unique_ptr<Logger> log;

    messageHandler_t messageHandler;
    std::string localNodeId;

    Result getRequest(std::string path);
    void getLocalNodeId();

public:
    DtndClient(const proxyConfig::DtnConfig& config, std::string loggerName = "dtn_lib");

    void setMessageHandler(messageHandler_t h);

    void onOpen();
    void onBundle(const std::string& bundle);

    bool registerEndpoint(std::string eid);
    void sendMessage(const std::vector<uint8_t>& payload);
};
