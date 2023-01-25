#pragma once
#include <memory>
#include <httplib.h>

class DtndClient
{
private:
    typedef struct
    {
        bool success;
        std::string content;
    } Result;

    std::unique_ptr<httplib::Client> http;

    Result getRequest(std::string path);

public:
    DtndClient(std::string address = "127.0.0.1", uint16_t port = 3000);

    bool registerEndpoint(std::string eid);
};
