#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <dtnd_client.hpp>

class DtnProxy : public rclcpp::Node
{
private:
    std::unique_ptr<DtndClient> dtn;

public:
    DtnProxy() : Node("dtn_proxy")
    {
        dtn = std::make_unique<DtndClient>();
        dtn->registerEndpoint("bla");
        RCLCPP_INFO(this->get_logger(), "DtnProxy up.");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DtnProxy>());
    rclcpp::shutdown();
    return 0;
}
