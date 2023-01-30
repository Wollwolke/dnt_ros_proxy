#pragma once
#include <httplib.h>

#include <logger.hpp>
#include <memory>
#include <ws_client.hpp>

class DtndClient {
private:
    using Result = struct Result {
        bool success;
        std::string content;
        Result(bool success, std::string content);
    };

    std::unique_ptr<httplib::Client> http;
    std::unique_ptr<WsClient> ws;
    std::unique_ptr<Logger> log;

    Result getRequest(std::string path);

public:
    DtndClient(std::string address = "127.0.0.1", uint16_t port = 3000,
               std::string loggerName = "dtn_lib");

    void onOpen();
    void onBundle(std::string bundle);

    bool registerEndpoint(std::string eid);
};
