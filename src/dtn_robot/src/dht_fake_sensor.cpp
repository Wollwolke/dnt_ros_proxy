#include <chrono>
#include <memory>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/detail/relative_humidity__struct.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/temperature.hpp>

namespace dtnrobot {

using namespace std::chrono_literals;

class DhtFake : public rclcpp::Node {
public:
    DhtFake() : Node("temp_sensor") {
        tempPublisher = this->create_publisher<sensor_msgs::msg::Temperature>("temperature", 10);
        humidityPublisher =
            this->create_publisher<sensor_msgs::msg::RelativeHumidity>("humidity", 10);
        timer = this->create_wall_timer(500ms, std::bind(&DhtFake::timerCallback, this));
        dist = std::uniform_real_distribution<>(0.7, 1.5);
    }

private:
    static constexpr auto TEMP_BASE = 20;
    static constexpr auto HUMIDITY_BASE = 50;

    std::default_random_engine generator;
    std::uniform_real_distribution<> dist;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr tempPublisher;
    rclcpp::Publisher<sensor_msgs::msg::RelativeHumidity>::SharedPtr humidityPublisher;
    sensor_msgs::msg::Temperature tempMsg;
    sensor_msgs::msg::RelativeHumidity humidityMsg;

    void timerCallback() {
        auto tmpTime = now();
        tempMsg.header.stamp = tmpTime;
        tempMsg.temperature = TEMP_BASE * dist(generator);

        humidityMsg.header.stamp = tmpTime;
        humidityMsg.relative_humidity = HUMIDITY_BASE * dist(generator);

        tempPublisher->publish(tempMsg);
        humidityPublisher->publish(humidityMsg);

        RCLCPP_DEBUG_STREAM(this->get_logger(), "Temp: " << tempMsg.temperature << " Humidity: "
                                                         << humidityMsg.relative_humidity);
    }
};

}  // namespace dtnrobot

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dtnrobot::DhtFake>());
    rclcpp::shutdown();
    return 0;
}
