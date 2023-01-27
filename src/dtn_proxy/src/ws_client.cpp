#include <functional>
#include <thread>
#include <ws_client.hpp>

WsClient::WsClient() {
    bundleHandler = [](std::string) {};
    openHandler = [] {};

    metadata.status = Status::UNKNOWN;

    // Disable logging
    endpoint.clear_access_channels(websocketpp::log::alevel::all);
    endpoint.clear_error_channels(websocketpp::log::elevel::all);

    endpoint.init_asio();
    endpoint.start_perpetual();  // Waits for connections

    thread.reset(new std::thread(&client::run, &endpoint));
}

WsClient::~WsClient() {
    endpoint.stop_perpetual();

    close(websocketpp::close::status::going_away);

    thread->join();
}

void WsClient::setBundleHandler(bundleHandler_t h) { bundleHandler = h; }

void WsClient::setOpenHandler(openHandler_t h) { openHandler = h; }

bool WsClient::connect(const std::string& uri) {
    websocketpp::lib::error_code errorCode;

    client::connection_ptr conReq = endpoint.get_connection(uri, errorCode);

    if (errorCode) {
        std::cout << "> Connect initialization error: " << errorCode.message()
                  << std::endl;
        return false;
    }

    metadata.hdl = conReq->get_handle();
    metadata.status = Status::CONNECTING;

    conReq->set_open_handler(
        std::bind(&WsClient::onOpen, this, &endpoint, std::placeholders::_1));
    conReq->set_fail_handler(
        std::bind(&WsClient::onFail, this, &endpoint, std::placeholders::_1));
    conReq->set_message_handler(std::bind(&WsClient::onMessage, this,
                                          std::placeholders::_1,
                                          std::placeholders::_2));
    conReq->set_close_handler(
        std::bind(&WsClient::onClose, this, &endpoint, std::placeholders::_1));

    endpoint.connect(conReq);

    return true;
}

void WsClient::close(websocketpp::close::status::value code) {
    if (metadata.status == Status::OPEN) {
        std::cout << "> Closing Websocket connection." << std::endl;

        websocketpp::lib::error_code errorCode;
        endpoint.close(metadata.hdl, code, "", errorCode);
        if (errorCode) {
            std::cout << "> Error initiating close: " << errorCode.message()
                      << std::endl;
        }
    }
}

void WsClient::send(std::string msg) {
    websocketpp::lib::error_code errorCode;

    endpoint.send(metadata.hdl, msg, websocketpp::frame::opcode::text,
                  errorCode);
    if (errorCode) {
        std::cout << "> Error sending message: " << errorCode.message()
                  << std::endl;
        return;
    }
}

void WsClient::onOpen(client* c, websocketpp::connection_hdl hdl) {
    metadata.status = Status::OPEN;
    openHandler();

    (void)c;
    (void)hdl;
    // TODO: check response header
    // TODO: logging
    // client::connection_ptr con = c->get_con_from_hdl(hdl);
    // server = con->get_response_header("Server");
    std::cout << metadata << std::endl;
}

void WsClient::onFail(client* c, websocketpp::connection_hdl hdl) {
    client::connection_ptr con = c->get_con_from_hdl(hdl);
    metadata.status = Status::FAILED;
    metadata.errorReason = con->get_ec().message();
    // TODO: check response header
    // TODO: logging
    // server = con->get_response_header("Server");
    std::cout << metadata << std::endl;
}

void WsClient::onClose(client* c, websocketpp::connection_hdl hdl) {
    // TODO: logging
    metadata.status = Status::CLOSED;
    client::connection_ptr con = c->get_con_from_hdl(hdl);
    std::stringstream s;
    s << "remote code: " << con->get_remote_close_code() << " ("
      << websocketpp::close::status::get_string(con->get_remote_close_code())
      << "), remote reason: " << con->get_remote_close_reason()
      << "\t-\t local code: " << con->get_local_close_code() << " ("
      << websocketpp::close::status::get_string(con->get_local_close_code())
      << "), local reason: " << con->get_local_close_reason();
    metadata.errorReason = s.str();
    std::cout << metadata << std::endl;
}

void WsClient::onMessage(websocketpp::connection_hdl, client::message_ptr msg) {
    // TODO: logging
    std::string payload;
    if (msg->get_opcode() == websocketpp::frame::opcode::TEXT) {
        payload = msg->get_payload();
    } else {
        payload = msg->get_payload();
        // payload = websocketpp::utility::to_hex(msg->get_payload());
    }
    std::cout << ">> " << payload << std::endl;

    bundleHandler(payload);
}

std::ostream& operator<<(std::ostream& out, const WsClient::Metadata& data) {
    out << "> Status: " << data.status << "\n"
        << "> Reason: "
        << (data.errorReason.empty() ? "N/A" : data.errorReason);
    return out;
}

std::ostream& operator<<(std::ostream& os, const WsClient::Status& status) {
    switch (status) {
        case WsClient::Status::UNKNOWN:
            return os << "Unknwon";
        case WsClient::Status::CLOSED:
            return os << "Closed";
        case WsClient::Status::CONNECTING:
            return os << "Connecting";
        case WsClient::Status::OPEN:
            return os << "Open";
        case WsClient::Status::FAILED:
            return os << "Failed";
            // omit default case to trigger compiler warning for missing cases
    };
    return os << static_cast<std::uint8_t>(status);
}
