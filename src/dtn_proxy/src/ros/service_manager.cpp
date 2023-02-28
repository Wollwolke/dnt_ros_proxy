#include "ros/service_manager.hpp"

#include <arpa/inet.h>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <string>
#include <vector>

#include "ros/dtn_msg_type.hpp"

namespace dtnproxy::ros {

using SharedFuture = std::shared_future<std::shared_ptr<rclcpp::SerializedMessage>>;

int ServiceManager::requestHeaderId(rmw_request_id_t requestHeader) {
    if (storedRequestHeaders.size() >= MAX_WAITING_REQUESTS) {
        return -1;
    }
    storedRequestHeaders.push_back(requestHeader);
    return storedRequestHeaders.size() - 1;
}

void ServiceManager::responseCallback(const std::string& topic, uint8_t requestId,
                                      std::shared_ptr<rclcpp::SerializedMessage> response) {
    // TODO: check stats recording

    std::vector<uint8_t> payload;
    auto rosMsgSize = buildDtnPayload(payload, response, requestId);
    if (stats) stats->rosReceived(topic, "unknown", rosMsgSize, DtnMsgType::RESPONSE);

    dtn->sendMessage(payload, topic, DtnMsgType::RESPONSE);
    if (stats) stats->dtnSent(topic, "unknown", payload.size(), DtnMsgType::RESPONSE);
}

void ServiceManager::requestCallback(const std::string& topic, const std::string& type,
                                     std::shared_ptr<rmw_request_id_t> requestHeader,
                                     std::shared_ptr<rclcpp::SerializedMessage> request) {
    // TODO: think about using the smart pointer here
    auto reqId = requestHeaderId(*requestHeader);
    if (-1 == reqId) {
        log->WARN() << "More than " << MAX_WAITING_REQUESTS << " open service requests. Ignoring.";
        return;
    }

    std::vector<uint8_t> payload;
    auto rosMsgSize = buildDtnPayload(payload, request, reqId);
    if (stats) stats->rosReceived(topic, type, rosMsgSize, DtnMsgType::REQUEST);

    dtn->sendMessage(payload, topic, DtnMsgType::REQUEST);
    if (stats) stats->dtnSent(topic, type, payload.size(), DtnMsgType::REQUEST);
}

ServiceManager::ServiceManager(rclcpp::Node& nodeHandle, conf::RosConfig config,
                               std::shared_ptr<DtndClient> dtn, const std::unique_ptr<Logger>& log)
    : ManagerBase(nodeHandle, config, dtn, log) {}

void ServiceManager::onDtnRequest(const std::string& topic, std::vector<uint8_t>& data,
                                  uint32_t size, uint8_t headerId) {
    rcl_serialized_message_t request{
        &data.front(),               // buffer
        size,                        // buffer_length
        size,                        // buffer_capacity
        rcl_get_default_allocator()  // allocator
    };
    auto sharedRequest = std::make_shared<rclcpp::SerializedMessage>(request);

    auto responseReceivedCallback = [this, topic, headerId](SharedFuture future) {
        const auto responseMsg = future.get();
        responseCallback(topic, headerId, responseMsg);
    };

    using namespace std::chrono_literals;
    while (!clients.at(topic)->wait_for_service(5s)) {
        log->INFO() << "Service " << topic << " not available, waiting...";
    }

    clients.at(topic)->async_send_request(sharedRequest, responseReceivedCallback);

    // TODO: find msgType in rosConfig
    if (stats) {
        stats->rosSent(topic, "unknown", static_cast<uint32_t>(size) + CDR_MSG_SIZE_OFFSET,
                       DtnMsgType::REQUEST);
    }
}

void ServiceManager::onDtnResponse(const std::string& topic, std::vector<uint8_t>& data,
                                   uint32_t size, uint8_t headerId) {
    rcl_serialized_message_t response{
        &data.front(),               // buffer
        size,                        // buffer_length
        size,                        // buffer_capacity
        rcl_get_default_allocator()  // allocator
    };
    auto sharedResponse = std::make_shared<rclcpp::SerializedMessage>(response);

    auto requestHeader = storedRequestHeaders.at(headerId);
    storedRequestHeaders.erase(storedRequestHeaders.begin() + headerId);

    servers.at(topic)->send_response(requestHeader, sharedResponse);

    // TODO: find msgType in rosConfig
    if (stats) {
        stats->rosSent(topic, "unknown", static_cast<uint32_t>(size) + CDR_MSG_SIZE_OFFSET,
                       DtnMsgType::RESPONSE);
    }
}

void ServiceManager::initServers() {
    auto serviceOptions = rcl_service_get_default_options();

    for (const auto& [topic, type] : config.servers) {
        auto advTopic = prefixTopic(topic, true);

        auto cb = std::bind(&ServiceManager::requestCallback, this, topic, type,
                            std::placeholders::_1, std::placeholders::_2);
        auto server = GenericService::make_shared(
            nodeHandle.get_node_base_interface()->get_shared_rcl_node_handle(), type, advTopic, cb,
            serviceOptions);

        // TODO: Check service group (nullptr)
        nodeHandle.get_node_services_interface()->add_service(server, nullptr);
        servers.insert_or_assign(topic, server);

        log->INFO() << "Providing service:\t" << advTopic;
    }
}

void ServiceManager::initClients() {
    auto clientOptions = rcl_client_get_default_options();

    for (const auto& [topic, type] : config.clients) {
        auto client = GenericClient::make_shared(nodeHandle.get_node_base_interface().get(),
                                                 nodeHandle.get_node_graph_interface(), topic, type,
                                                 clientOptions);

        // TODO: Check service group (nullptr)
        nodeHandle.get_node_services_interface()->add_client(client, nullptr);
        clients.insert_or_assign(topic, client);

        using namespace std::chrono_literals;
        if (!client->wait_for_service(1s)) {
            log->WARN() << "Service " << topic << " not available!";
        }
    }
}

}  // namespace dtnproxy::ros
