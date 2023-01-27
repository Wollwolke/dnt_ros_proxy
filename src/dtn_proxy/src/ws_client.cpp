#include <functional>
#include <thread>
#include <ws_client.hpp>

WsClient::WsClient() {
    bundleHandler = [](std::string) {};

    // Disable logging
    endpoint.clear_access_channels(websocketpp::log::alevel::all);
    endpoint.clear_error_channels(websocketpp::log::elevel::all);

    endpoint.init_asio();
    endpoint.start_perpetual();  // Waits for connections

    thread.reset(new std::thread(&client::run, &endpoint));
}

WsClient::~WsClient() {
    endpoint.stop_perpetual();

    if (connection->getStatus() == "Open") {
        // Only close open connections
        std::cout << "> Closing Websocket connection." << std::endl;
        close(websocketpp::close::status::going_away);
    }

    thread->join();
}

bool WsClient::connect(const std::string& uri) {
    websocketpp::lib::error_code errorCode;

    client::connection_ptr connectionRequest =
        endpoint.get_connection(uri, errorCode);

    if (errorCode) {
        std::cout << "> Connect initialization error: " << errorCode.message()
                  << std::endl;
        return false;
    }

    connection = std::make_shared<ConnectionDetails>(
        connectionRequest->get_handle(), uri);

    connectionRequest->set_open_handler(std::bind(&ConnectionDetails::onOpen,
                                                  connection, &endpoint,
                                                  std::placeholders::_1));
    connectionRequest->set_fail_handler(std::bind(&ConnectionDetails::onFail,
                                                  connection, &endpoint,
                                                  std::placeholders::_1));
    connectionRequest->set_message_handler(
        std::bind(&ConnectionDetails::onMessage, connection,
                  websocketpp::lib::placeholders::_1,
                  websocketpp::lib::placeholders::_2));

    endpoint.connect(connectionRequest);

    return true;
}

void WsClient::close(websocketpp::close::status::value code) {
    websocketpp::lib::error_code ec;

    endpoint.close(connection->getHandle(), code, "", ec);
    if (ec) {
        std::cout << "> Error initiating close: " << ec.message() << std::endl;
    }
}

void WsClient::send(std::string msg) {
    websocketpp::lib::error_code ec;

    endpoint.send(connection->getHandle(), msg,
                  websocketpp::frame::opcode::text, ec);
    if (ec) {
        std::cout << "> Error sending message: " << ec.message() << std::endl;
        return;
    }
}

void WsClient::setBundleHandler(bundleHandler_t h) { bundleHandler = h; }

WsClient::ConnectionDetails::ConnectionDetails(websocketpp::connection_hdl hdl,
                                               std::string uri)
    : hdl(hdl), status("Connecting"), uri(uri), server("N/A") {}

void WsClient::ConnectionDetails::onOpen(client* c,
                                         websocketpp::connection_hdl hdl) {
    status = "Open";
    client::connection_ptr con = c->get_con_from_hdl(hdl);
    server = con->get_response_header("Server");
    std::cout << *this << std::endl;
}

void WsClient::ConnectionDetails::onFail(client* c,
                                         websocketpp::connection_hdl hdl) {
    status = "Failed";
    client::connection_ptr con = c->get_con_from_hdl(hdl);
    server = con->get_response_header("Server");
    errorReason = con->get_ec().message();
    std::cout << *this << std::endl;
}

void WsClient::ConnectionDetails::onClose(client* c,
                                          websocketpp::connection_hdl hdl) {
    status = "Closed";
    client::connection_ptr con = c->get_con_from_hdl(hdl);
    std::stringstream s;
    s << "close code: " << con->get_remote_close_code() << " ("
      << websocketpp::close::status::get_string(con->get_remote_close_code())
      << "), close reason: " << con->get_remote_close_reason();
    errorReason = s.str();
}

// TODO: implement this
void WsClient::ConnectionDetails::onMessage(websocketpp::connection_hdl,
                                            client::message_ptr msg) {
    if (msg->get_opcode() == websocketpp::frame::opcode::TEXT) {
        std::cout << msg->get_payload();
    } else {
        std::cout << websocketpp::utility::to_hex(msg->get_payload());
    }
}

websocketpp::connection_hdl WsClient::ConnectionDetails::getHandle() {
    return hdl;
}

std::string WsClient::ConnectionDetails::getStatus() { return status; }

std::ostream& operator<<(std::ostream& out,
                         const WsClient::ConnectionDetails& data) {
    out << "> URI: " << data.uri << "\n"
        << "> Status: " << data.status << "\n"
        << "> Remote Server: "
        << (data.server.empty() ? "None Specified" : data.server) << "\n"
        << "> Error/close reason: "
        << (data.errorReason.empty() ? "N/A" : data.errorReason);
    return out;
}
