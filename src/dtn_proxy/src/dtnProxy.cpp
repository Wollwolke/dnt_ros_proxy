#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <functional>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
// #include <rclcpp/serialization.hpp>
// #include <rcpputils/shared_library.hpp>
// #include <rosbag2_cpp/typesupport_helpers.hpp>
// #include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <vector>

#include "configuration.hpp"
#include "dtnd_client.hpp"
#include "logger.hpp"
#include "ws_datatypes.hpp"

#define PACKAGE_NAME "dtn_proxy"

class DtnProxy : public rclcpp::Node {
private:
    proxyConfig::Config config;

    std::unique_ptr<DtndClient> dtn;

    std::map<std::string, std::shared_ptr<rclcpp::GenericSubscription>> subscriber;
    std::map<std::string, std::shared_ptr<rclcpp::GenericPublisher>> publisher;

    void initPublisher() {
        rclcpp::QoS qos = rclcpp::QoS(10);

        for (auto& [topic, type] : config.ros.pubTopics) {
            publisher.insert_or_assign(topic, this->create_generic_publisher(topic, type, qos));
        }
    }

    void initSubscriber() {
        rclcpp::QoS qos = rclcpp::QoS(10);

        for (auto& [topic, type] : config.ros.subTopics) {
            auto cb = std::bind(&DtnProxy::topicCallback, this, topic, std::placeholders::_1);
            subscriber.insert_or_assign(topic,
                                        this->create_generic_subscription(topic, type, qos, cb));
        }
    }

    void fatalShutdown(const std::string& reason) {
        RCLCPP_FATAL(this->get_logger(), reason.c_str());
        // TODO: how to properly shut down?!?
        rclcpp::shutdown();
    }

    void loadConfig() {
        auto packageShareDirectory = ament_index_cpp::get_package_share_directory(PACKAGE_NAME);
        auto parameterDesc = rcl_interfaces::msg::ParameterDescriptor{};
        parameterDesc.description = "Absolute path to the configuration file.";

        this->declare_parameter("configurationPath", packageShareDirectory + "/config/node0.tomsl",
                                parameterDesc);
        auto path =
            this->get_parameter("configurationPath").get_parameter_value().get<std::string>();

        try {
            config = proxyConfig::ConfigurationReader::readConfigFile(path, this->get_name());
        } catch (proxyConfig::ConfigException& e) {
            fatalShutdown(e.what());
        }
    }

    void topicCallback(const std::string& topic, std::shared_ptr<rclcpp::SerializedMessage> msg) {
        auto cdrMsg = msg->get_rcl_serialized_message();
        auto len = static_cast<uint32_t>(cdrMsg.buffer_length);

        // hopefully resolves endianness problem
        len = htonl(len);
        uint8_t* bytePointer = reinterpret_cast<uint8_t*>(&len);
        std::vector<uint8_t> payload(bytePointer, bytePointer + sizeof(len));

        payload.insert(payload.end(), cdrMsg.buffer, cdrMsg.buffer + cdrMsg.buffer_length);

        dtn->sendMessage(payload, topic);
    }

    // TODO: Use this dynamic type support stuff to get the header...
    // void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) {
    //     using MessageT = geometry_msgs::msg::PoseStamped;
    //     MessageT string_msg;

    //     auto msg_type = "geometry_msgs/msg/PoseStamped";
    //     auto lib = rosbag2_cpp::get_typesupport_library(
    //         msg_type, "rosidl_typesupport_cpp");
    //     auto typesupport = rosbag2_cpp::get_typesupport_handle(
    //         msg_type, "rosidl_typesupport_cpp", lib);

    //     auto cap = msg->capacity();
    //     auto size = msg->size();
    //     auto test = msg->get_rcl_serialized_message();
    //     auto serializer = rclcpp::SerializationBase(typesupport);
    //     serializer.deserialize_message(msg.get(), &string_msg);
    //     auto header_timestamp =
    //         message_filters::message_traits::HasHeader<MessageT>::value;
    //     RCLCPP_INFO(this->get_logger(), "I heard: %s",
    //     string_msg.data.c_str());
    // }

    // void test(std::string type) {
    //     auto lib = rosbag2_cpp::get_typesupport_library(type, "rosidl_typesupport_cpp");
    //     auto typesupport = rosbag2_cpp::get_typesupport_handle(type, "rosidl_typesupport_cpp",
    //     lib);
    // }

public:
    DtnProxy() : Node("dtn_proxy") {
        // TODO: cleanup includes

        loadConfig();
        dtn = std::make_unique<DtndClient>(config.dtn, this->get_name());
        dtn->setMessageHandler(std::bind(&DtnProxy::onDtnMessage, this, std::placeholders::_1));
        // TODO: check order of init / introduce ready flag (return values)
        initPublisher();

        std::vector<std::string> topics;
        std::transform(publisher.begin(), publisher.end(), std::back_inserter(topics),
                       [](auto const& pubMap) { return pubMap.first; });

        if (!dtn->connect(topics)) fatalShutdown("Error connecting to dtnd!");
        initSubscriber();

        RCLCPP_INFO_STREAM(this->get_logger(), "DtnProxy up.");
    }

    void onDtnMessage(const data::WsReceive& bundle) {
        const uint8_t SIZE_BYTES = 4;
        auto data = bundle.data;
        auto topic = bundle.dst;
        topic = topic.substr(topic.rfind("/") + 1);

        // Get cdr buffer size from first 4 bytes
        uint32_t size;
        memcpy(&size, &data.front(), SIZE_BYTES);
        size = ntohl(size);

        std::vector<uint8_t> buffer(data.begin() + SIZE_BYTES, data.end());

        rcl_serialized_message_t cdrMsg{
            &buffer.front(),             // buffer
            size,                        // buffer_length
            size,                        // buffer_capacity
            rcl_get_default_allocator()  // allocator
        };

        publisher.at(topic)->publish(rclcpp::SerializedMessage(cdrMsg));
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DtnProxy>());
    rclcpp::shutdown();
    return 0;
}
