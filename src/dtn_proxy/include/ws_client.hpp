#pragma once
#include <functional>
#include <map>
#include <memory>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>

class WsClient {
private:
    enum Status : uint8_t;

    using client = websocketpp::client<websocketpp::config::asio_client>;
    using bundleHandler_t = std::function<void(std::string)>;
    using openHandler_t = std::function<void()>;

    using Metadata = struct Metadata {
        websocketpp::connection_hdl hdl;
        Status status;
        std::string errorReason;
    };

    bundleHandler_t bundleHandler;
    openHandler_t openHandler;
    client endpoint;
    Metadata metadata;
    std::shared_ptr<websocketpp::lib::thread> thread;

public:
    WsClient();
    ~WsClient();

    void setBundleHandler(bundleHandler_t h);
    void setOpenHandler(openHandler_t h);

    bool connect(const std::string& uri);
    void close(websocketpp::close::status::value code);
    void send(std::string msg);

    void onOpen(client* c, websocketpp::connection_hdl hdl);
    void onFail(client* c, websocketpp::connection_hdl hdl);
    void onClose(client* c, websocketpp::connection_hdl hdl);
    void onMessage(websocketpp::connection_hdl hdl, client::message_ptr msg);

    friend std::ostream& operator<<(std::ostream& out, const Metadata& data);
    friend std::ostream& operator<<(std::ostream& os, const Status& status);
};

enum WsClient::Status : uint8_t { UNKNOWN, CLOSED, CONNECTING, OPEN, FAILED };