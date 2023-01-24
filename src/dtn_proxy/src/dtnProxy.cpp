#include "rclcpp/rclcpp.hpp"

#include <httplib.h>

class DtnProxy : public rclcpp::Node
{
public:
    DtnProxy() : Node("dtn_proxy")
    {
        RCLCPP_INFO(this->get_logger(), "DtnProxy up.");
    }
};

using namespace std;

int main(int argc, char *argv[])
{
    httplib::Client cli("localhost", 3000);
    if (auto res = cli.Get("/status/nodeidd"))
    {
        cout << res->status << endl;
        cout << res->get_header_value("Content-Type") << endl;
        cout << res->body << endl;
    }
    else
    {
        cout << "error code: " << res.error() << std::endl;
    }
        // rclcpp::init(argc, argv);
        // rclcpp::spin(std::make_shared<DtnProxy>());
        // rclcpp::shutdown();
        return 0;
    }
