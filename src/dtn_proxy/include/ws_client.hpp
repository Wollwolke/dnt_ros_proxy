#pragma once
#include <functional>
#include <map>
#include <memory>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>

class WsClient {
private:
    using client = websocketpp::client<websocketpp::config::asio_client>;
    using bundleHandler_t = std::function<void(std::string)>;

    class ConnectionDetails;

    bundleHandler_t bundleHandler;
    client endpoint;
    std::shared_ptr<ConnectionDetails> connection;
    std::shared_ptr<websocketpp::lib::thread> thread;

public:
    WsClient();
    ~WsClient();
    bool connect(const std::string& uri);
    void close(websocketpp::close::status::value code);
    void send(std::string msg);
    
    void setBundleHandler(bundleHandler_t h);

    friend std::ostream& operator<<(std::ostream& out,
                                    const ConnectionDetails& data);
};

class WsClient::ConnectionDetails {
private:
    websocketpp::connection_hdl hdl;
    std::string status;
    std::string uri;
    std::string server;
    std::string errorReason;

public:
    using client = websocketpp::client<websocketpp::config::asio_client>;

    ConnectionDetails(websocketpp::connection_hdl hdl, std::string uri);

    void onOpen(client* c, websocketpp::connection_hdl hdl);
    void onFail(client* c, websocketpp::connection_hdl hdl);
    void onClose(client* c, websocketpp::connection_hdl hdl);
    void onMessage(websocketpp::connection_hdl hdl, client::message_ptr msg);

    websocketpp::connection_hdl getHandle();
    std::string getStatus();

    friend std::ostream& operator<<(std::ostream& out,
                                    const ConnectionDetails& data);
};
