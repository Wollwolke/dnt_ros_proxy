#include "dtnd_client.hpp"

#include "ws_datatypes.hpp"

using std::string;

DtndClient::DtndClient(const proxyConfig::DtnConfig& config, std::string loggerName)
    : config(config) {
    messageHandler = [](const std::vector<uint8_t>) {};

    http = std::make_unique<httplib::Client>(config.address, config.port);
    ws = std::make_unique<WsClient>(loggerName);
    log = std::make_unique<Logger>(loggerName, "dtn");

    getLocalNodeId();

    ws->setOpenHandler(std::bind(&DtndClient::onOpen, this));
    ws->setBundleHandler(std::bind(&DtndClient::onBundle, this, std::placeholders::_1));
}

void DtndClient::setMessageHandler(messageHandler_t h) { messageHandler = h; }

void DtndClient::onOpen() {
    log->INFO() << "WS Connection to dtnd opened.";
    ws->send("/data");
    ws->send("/subscribe bla");
}

void DtndClient::onBundle(const std::string& bundle) {
    log->INFO() << "Got bundle";

    nlohmann::json j = nlohmann::json::from_cbor(bundle);
    log->DBG() << j;

    auto payload = j.get<data::Dtn2Ws>().data;
    messageHandler(payload);
}

DtndClient::Result::Result(bool success = false, string content = "")
    : success(success), content(content) {}

bool DtndClient::registerEndpoint(string eid) {
    auto result = getRequest("/register?" + eid);
    ws->connect("ws://localhost:3000/ws");
    log->DBG() << result.content;
    return result.success;
}

void DtndClient::sendMessage(const std::vector<uint8_t>& payload) {
    const auto MS_IN_MINUTE = 60 * 1000;

    data::Ws2Dtn msg{
        localNodeId,                     // std::string src,
        config.remoteNodeId + "/bla",    // std::string dst,
        false,                           // bool delivery_notification,
        config.lifetime * MS_IN_MINUTE,  // uint64_t lifetime,
        payload,                         // std::vector<uint8_t>& data,
    };
    nlohmann::json jsonMsg = msg;
    std::vector<uint8_t> cborMsg = nlohmann::json::to_cbor(jsonMsg);
    ws->send(cborMsg);
}

DtndClient::Result DtndClient::getRequest(string path) {
    using namespace std::literals::string_literals;
    string content = "";
    if (auto res = http->Get(path)) {
        if (res->status == 200)
            return Result(true, res->body);
        else
            content = "HTTP return code: "s + std::to_string(res->status) + " - "s + res->reason;
    } else
        content = "HTTP Client Error: "s + to_string(res.error());

    return Result(false, content);
}

void DtndClient::getLocalNodeId() {
    auto result = getRequest("/status/nodeid");
    if (result.success) {
        localNodeId = result.content;
    } else {
        log->ERR() << "Requesting nodeId: " << result.content;
    }
}
