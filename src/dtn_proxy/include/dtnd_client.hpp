#pragma once
#include <httplib.h>

#include <memory>
#include <ws_client.hpp>

class DtndClient {
private:
    typedef struct Result {
        bool success;
        std::string content;
        Result(bool success, std::string content);
    } Result;

    std::unique_ptr<httplib::Client> http;
    std::unique_ptr<WsClient> ws;

    Result getRequest(std::string path);

public:
    DtndClient(std::string address = "127.0.0.1", uint16_t port = 3000);

    bool registerEndpoint(std::string eid);
};
