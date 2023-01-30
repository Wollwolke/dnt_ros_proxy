#include <dtnd_client.hpp>
#include <logger.hpp>
#include <memory>

#include "rclcpp/rclcpp.hpp"

class DtnProxy : public rclcpp::Node {
private:
    std::unique_ptr<DtndClient> dtn;

public:
    DtnProxy() : Node("dtn_proxy") {
        dtn = std::make_unique<DtndClient>("127.0.0.1", 3000, "dtn_proxy");
        dtn->registerEndpoint("bla");
        RCLCPP_INFO_STREAM(this->get_logger(), "DtnProxy up.");
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DtnProxy>());
    rclcpp::shutdown();
    return 0;
}
