#include <dtnd_client.hpp>
#include <functional>
#include <logger.hpp>
#include <map>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
// #include "rcpputils/shared_library.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

class DtnProxy : public rclcpp::Node {
private:
    std::unique_ptr<DtndClient> dtn;

    std::map<std::string, std::string> subTopics;
    std::map<std::string, std::string> pubTopics;
    std::map<std::string, std::shared_ptr<rclcpp::GenericSubscription>> subscriber;
    std::map<std::string, std::shared_ptr<rclcpp::GenericPublisher>> publisher;

    void initRosInterface() {
        rclcpp::QoS qos = rclcpp::QoS(10);

        // Subscribers
        auto cb = std::bind(&DtnProxy::topicCallback, this, std::placeholders::_1);
        for (auto& [topic, type] : subTopics) {
            subscriber.insert_or_assign(topic,
                                        this->create_generic_subscription(topic, type, qos, cb));
        }

        // Publisher
        for (auto& [topic, type] : pubTopics) {
            publisher.insert_or_assign(topic, this->create_generic_publisher(topic, type, qos));
        }
    }

    void loadConfig() {
        subTopics.insert_or_assign("test", "geometry_msgs/msg/PoseStamped");
        pubTopics.insert_or_assign("echo", "geometry_msgs/msg/PoseStamped");
    }

    void topicCallback(std::shared_ptr<rclcpp::SerializedMessage> msg) {
        auto cdrMsg = msg->get_rcl_serialized_message();
        auto len = static_cast<uint32_t>(cdrMsg.buffer_length);

        // hopefully resolves endianness problem
        len = htonl(len);
        uint8_t* bytePointer = reinterpret_cast<uint8_t*>(&len);
        std::vector<uint8_t> payload(bytePointer, bytePointer + sizeof(len));

        payload.insert(payload.end(), cdrMsg.buffer, cdrMsg.buffer + cdrMsg.buffer_length);
        dtn->sendMessage(payload);
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
        // TODO: logger as ref not name
        dtn = std::make_unique<DtndClient>("127.0.0.1", 3000, this->get_name());
        dtn->setMessageHandler(std::bind(&DtnProxy::onDtnMessage, this, std::placeholders::_1));
        dtn->registerEndpoint("bla");

        loadConfig();
        initRosInterface();
        RCLCPP_INFO_STREAM(this->get_logger(), "DtnProxy up.");
    }

    void onDtnMessage(const std::vector<uint8_t>& data) {
        const std::string topic = "echo";
        const uint8_t SIZE_BYTES = 4;

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
