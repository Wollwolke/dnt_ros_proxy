#pragma once

#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "ros/generic_client.hpp"
#include "ros/generic_service.hpp"
#include "ros/manager_base.hpp"

namespace dtnproxy::ros {

class ServiceManager : public ManagerBase {
private:
    // ! Change Bytes in 'ManagerBase::addHeaderToDtnPayload'
    static constexpr auto MAX_WAITING_REQUESTS = 256;

    std::map<std::string, std::shared_ptr<GenericClient>> clients;
    std::map<std::string, std::shared_ptr<GenericService>> servers;

    std::vector<rmw_request_id_t> storedRequestHeaders;

    int requestHeaderId(rmw_request_id_t requestHeader);

    void responseCallback(const std::string& topic, uint8_t requestId,
                          std::shared_ptr<rclcpp::SerializedMessage> response);
    void requestCallback(const std::string& topic, const std::string& type,
                         std::shared_ptr<rmw_request_id_t> requestHeader,
                         std::shared_ptr<rclcpp::SerializedMessage> request);

public:
    ServiceManager();
    ServiceManager(rclcpp::Node& nodeHandle, conf::RosConfig config,
                   std::shared_ptr<DtndClient> dtn, const std::unique_ptr<Logger>& log);

    void onDtnRequest(const std::string& topic, std::vector<uint8_t>& data, uint32_t size,
                      uint8_t headerId);
    void onDtnResponse(const std::string& topic, std::vector<uint8_t>& data, uint32_t size,
                       uint8_t headerId);
    void initServers();
    void initClients();
};

}  // namespace dtnproxy::ros
