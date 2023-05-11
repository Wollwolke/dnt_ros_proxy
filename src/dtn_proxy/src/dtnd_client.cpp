#include "dtnd_client.hpp"

#include <nlohmann/json.hpp>

namespace dtnproxy {

DtndClient::DtndClient(const conf::DtnConfig& config) : config(config) {
    messageHandler = [](const data::WsReceive&) {};
    endpointsToRegister.emplace_back("remoteConfig", ros::DtnMsgType::INTERNAL);

    http = std::make_unique<httplib::Client>(config.address, config.port);
    ws = std::make_unique<WsClient>();
    log = std::make_unique<Logger>("dtn");

    ws->setConnectionStatusHandler(
        std::bind(&DtndClient::onConnectionStatus, this, std::placeholders::_1));
    ws->setBundleHandler(std::bind(&DtndClient::onBundle, this, std::placeholders::_1));
    ws->connect("ws://" + config.address + ":" + std::to_string(config.port) + "/ws");
}

void DtndClient::setMessageHandler(messageHandler_t h) { messageHandler = h; }

void DtndClient::onConnectionStatus(const bool success) {
    std::unique_lock<std::mutex> lock(stateMutex);
    if (success) {
        wsStatus = WsState::CONNECTED;
        lock.unlock();
        log->INFO() << "WS Connection to dtnd opened.";
        registerKnownEndpoints();
        sendRemoteConfig();
    } else {
        wsStatus = WsState::ERROR;
        log->WARN() << "WS Connection to dtnd failed.";
    }
}

void DtndClient::onBundle(const std::string& bundle) {
    nlohmann::json j = nlohmann::json::from_cbor(bundle);
    log->DBG() << j;

    auto payload = j.get<data::WsReceive>();
    messageHandler(payload);
}

DtndClient::Result::Result(bool success, std::string content)
    : success(success), content(content) {}

bool DtndClient::registerSubscribeEndpoints() {
    std::lock_guard<std::mutex> lock(endpointsMutex);

    for (auto& [eid, type] : endpointsToRegister) {
        auto typedEndpoint = eid;
        buildEndpointId(typedEndpoint, type);

        auto result = getRequest("/register?" + typedEndpoint);
        log->DBG() << result.content;
        if (!result.success) return false;

        ws->send("/subscribe " + typedEndpoint);
    }
    return true;
}

void DtndClient::buildEndpointId(std::string& endpoint, ros::DtnMsgType type) {
    using Type = ros::DtnMsgType;

    switch (type) {
        case Type::TOPIC:
            endpoint.insert(0, "rt_");
            break;
        case Type::REQUEST:
            endpoint.insert(0, "rq_");
            break;
        case Type::RESPONSE:
            endpoint.insert(0, "rr_");
            break;
        case Type::INTERNAL:
            endpoint.insert(0, "proxy_");
            break;
        case Type::INVALID:
        default:
            return;
    }
}

void DtndClient::registerKnownEndpoints() {
    {
        std::lock_guard<std::mutex> lock(stateMutex);
        if (WsState::CONNECTED != wsStatus) {
            log->FATAL() << "Can't register endpionts, not connection to dtnd!";
            return;
        }
    }

    // Set WS to cbor data mode
    ws->send("/data");

    bool ok = true;
    ok &= getLocalNodeId();
    ok &= registerSubscribeEndpoints();
    if (!ok) {
        log->FATAL() << "Error registering endopints!";
    }
}

void DtndClient::registerEndpoints(const std::vector<DtnEndpoint>& endpoints) {
    {
        std::lock_guard<std::mutex> lock(endpointsMutex);
        endpointsToRegister.insert(endpointsToRegister.end(), endpoints.begin(), endpoints.end());
    }
    registerKnownEndpoints();
}

void DtndClient::sendMessage(const Message& dtnMsg) {
    {
        std::lock_guard<std::mutex> lock(stateMutex);
        if (WsState::CONNECTED != wsStatus) {
            log->FATAL() << "Can't send message, not connection to dtnd!";
            return;
        }
    }

    const auto MS_IN_SECOND = 1000;

    auto typedEndpoint = dtnMsg.endpoint;
    buildEndpointId(typedEndpoint, dtnMsg.msgType);

    auto lifetime = (dtnMsg.lifetime == 0) ? config.lifetime : dtnMsg.lifetime;

    data::WsSend msg{
        localNodeId,                                // std::string src,
        config.remoteNodeId + "/" + typedEndpoint,  // std::string dst,
        lifetime * MS_IN_SECOND,                    // uint64_t lifetime,
        dtnMsg.bundleFlags,                         // uint64_t bundle_flags,
        dtnMsg.payload,                             // std::vector<uint8_t>& data,
    };
    std::vector<uint8_t> cborMsg = nlohmann::json::to_cbor(msg);
    ws->send(std::move(cborMsg));
}

void DtndClient::sendRemoteConfig(std::vector<uint8_t> config) {
    std::lock_guard<std::mutex> lock(remoteConfigMutex);
    if (!config.empty()) {
        latestRemoteConfig = std::move(config);
    }

    // TODO: find appropriate lifetime
    if (!latestRemoteConfig.empty()) {
        Message msg = {latestRemoteConfig, "remoteConfig", ros::DtnMsgType::INTERNAL,
                       BundleFlags::BUNDLE_REMOVE_OLDER_BUNDLES, 99999};
        sendMessage(msg);
    }
}

DtndClient::Result DtndClient::getRequest(std::string path) {
    using namespace std::literals::string_literals;
    std::string content = "";
    if (auto res = http->Get(path)) {
        if (res->status == 200) {
            return Result(true, res->body);
        } else {
            content = "HTTP return code: "s + std::to_string(res->status) + " - "s + res->reason;
        }
    } else {
        content = "HTTP Client Error: "s + to_string(res.error());
    }

    return Result(false, content);
}

bool DtndClient::getLocalNodeId() {
    auto result = getRequest("/status/nodeid");
    if (result.success) {
        localNodeId = result.content;
        return true;
    } else {
        log->ERR() << "Requesting nodeId: " << result.content;
        return false;
    }
}

}  // namespace dtnproxy
