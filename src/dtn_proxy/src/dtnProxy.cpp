#include "rclcpp/rclcpp.hpp"

class DtnProxy : public rclcpp::Node
{
public:
    DtnProxy() : Node("dtn_proxy")
    {
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
