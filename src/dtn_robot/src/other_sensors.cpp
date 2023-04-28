#include <chrono>
#include <memory>
#include <radiation_msgs/msg/detail/dose__struct.hpp>
#include <radiation_msgs/msg/detail/dose_rate__struct.hpp>
#include <radiation_msgs/msg/dose_rate.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>

namespace dtnrobot {

using namespace std::chrono_literals;

class OtherSensors : public rclcpp::Node {
public:
    OtherSensors() : Node("other_sensors") {
        generator = std::default_random_engine(SEED);
        pressureDist = std::uniform_real_distribution<>(1003, 1004);
        radiationDist = std::uniform_int_distribution<>(10, 60);

        pressurePublisher =
            this->create_publisher<sensor_msgs::msg::FluidPressure>("atmPressure", 10);
        radiationPublisher = this->create_publisher<radiation_msgs::msg::DoseRate>("radiation", 10);

        pressureMsg.header.frame_id = "imu_link";
        radiationMsg.header.frame_id = "imu_link";
        radiationMsg.radiation_type = 7;  // Alpha, beta, gamma
        radiationMsg.units = 0;
        radiationMsg.integration_time = 120;  // window of 120s

        shortTimer = this->create_wall_timer(500ms, std::bind(&OtherSensors::shortTimerCb, this));
        longTimer = this->create_wall_timer(1s, std::bind(&OtherSensors::longTimerCb, this));
    }

private:
    static constexpr auto SEED = 8779767569;

    std::default_random_engine generator;
    std::uniform_real_distribution<double> pressureDist;
    std::uniform_int_distribution<> radiationDist;

    rclcpp::TimerBase::SharedPtr shortTimer;
    rclcpp::TimerBase::SharedPtr longTimer;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressurePublisher;
    rclcpp::Publisher<radiation_msgs::msg::DoseRate>::SharedPtr radiationPublisher;

    sensor_msgs::msg::FluidPressure pressureMsg;
    radiation_msgs::msg::DoseRate radiationMsg;

    void shortTimerCb() {
        auto timeNow = now();

        pressureMsg.header.stamp = timeNow;
        pressureMsg.fluid_pressure = pressureDist(generator) * 100;  // hPa -> Pa
        pressurePublisher->publish(pressureMsg);
    }

    void longTimerCb() {
        auto timeNow = now();

        radiationMsg.header.stamp = timeNow;
        radiationMsg.rate = radiationDist(generator) / 60.0;  // cpm -> rate per second
        radiationPublisher->publish(radiationMsg);
    }
};

}  // namespace dtnrobot

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dtnrobot::OtherSensors>());
    rclcpp::shutdown();
    return 0;
}
