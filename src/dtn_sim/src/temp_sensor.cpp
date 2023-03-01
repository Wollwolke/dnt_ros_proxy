#include <chrono>
#include <memory>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>

namespace dtnsim {

using namespace std::chrono_literals;

class TempSensor : public rclcpp::Node {
public:
    TempSensor() : Node("temp_sensor") {
        publisher = this->create_publisher<sensor_msgs::msg::Temperature>("temperature", 10);
        timer = this->create_wall_timer(500ms, std::bind(&TempSensor::timer_callback, this));
        dist = std::uniform_real_distribution<>(8.0, 36.0);
    }

private:
    std::default_random_engine generator;
    std::uniform_real_distribution<> dist;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr publisher;
    sensor_msgs::msg::Temperature msg;

    void timer_callback() {
        msg.header.stamp = now();
        msg.temperature = dist(generator);
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Current Temperatur: " << msg.temperature);
        publisher->publish(msg);
    }
};

}  // namespace dtnsim

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dtnsim::TempSensor>());
    rclcpp::shutdown();
    return 0;
}
