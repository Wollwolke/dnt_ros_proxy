#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <memory>
#include <random>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

namespace dtnrobot {

using namespace std::chrono_literals;

class StatusPublisher : public rclcpp::Node {
public:
    StatusPublisher() : Node("temp_sensor") {
        initMsgs();

        tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

        timer = this->create_wall_timer(1s, std::bind(&StatusPublisher::timer_callback, this));
        batteryPublisher =
            this->create_publisher<sensor_msgs::msg::BatteryState>("status/battery", 10);
        diagPublisher =
            this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("status/tempSensor", 10);
        posPublisher =
            this->create_publisher<geometry_msgs::msg::PointStamped>("status/position", 10);
        bernoulli_025 = std::bernoulli_distribution(0.025);
        bernoulli_1 = std::bernoulli_distribution(0.01);
    }

private:
    static constexpr auto BATT_MAX_V = 4.2;
    static constexpr auto BATT_MIN_V = 3.0;

    std::default_random_engine generator;
    std::bernoulli_distribution bernoulli_025;
    std::bernoulli_distribution bernoulli_1;

    rclcpp::TimerBase::SharedPtr timer;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;

    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr batteryPublisher;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagPublisher;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr posPublisher;

    sensor_msgs::msg::BatteryState batteryMsg;
    diagnostic_msgs::msg::DiagnosticArray diagArray;
    diagnostic_msgs::msg::DiagnosticStatus diagMsg;
    geometry_msgs::msg::PointStamped posMsg;

    void updateBatteryMsg() {
        batteryMsg.header.stamp = now();
        if (batteryMsg.percentage > 0.01 && bernoulli_025(generator)) {
            batteryMsg.percentage -= 0.01;
            batteryMsg.voltage = BATT_MIN_V + (batteryMsg.percentage * (BATT_MAX_V - BATT_MIN_V));
        }
    }

    void updateDiagMsg() {
        static auto brokenIterations = 0;
        if (diagnostic_msgs::msg::DiagnosticStatus::OK == diagMsg.level) {
            if (bernoulli_1(generator)) {
                diagMsg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            }
        } else if (brokenIterations >= 7) {
            diagMsg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            brokenIterations = 0;
        } else {
            brokenIterations++;
        }

        diagArray.header.stamp = now();
        diagArray.status.clear();
        diagArray.status.push_back(diagMsg);
    }

    void updatePosMsg() {
        try {
            auto transform = tfBuffer->lookupTransform("map", "base_link", tf2::TimePointZero);
            posMsg.point.x = transform.transform.translation.x;
            posMsg.point.y = transform.transform.translation.y;
            posMsg.header.stamp = now();
        } catch (const tf2::TransformException& ex) {
            auto& clk = *this->get_clock();
            RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), clk, 5000,
                                        "Could not transform map to base_link: " << ex.what());
            return;
        }
    }

    void timer_callback() {
        updateBatteryMsg();
        updateDiagMsg();
        updatePosMsg();

        batteryPublisher->publish(batteryMsg);
        diagPublisher->publish(diagArray);
        posPublisher->publish(posMsg);
    }

    void initMsgs() {
        batteryMsg.header.stamp = now();
        batteryMsg.voltage = BATT_MAX_V;
        batteryMsg.temperature = float{NAN};
        batteryMsg.current = float{NAN};
        batteryMsg.charge = float{NAN};
        batteryMsg.capacity = float{NAN};
        batteryMsg.design_capacity = float{NAN};
        batteryMsg.percentage = 1.0;
        batteryMsg.power_supply_status =
            sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        batteryMsg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
        batteryMsg.power_supply_technology =
            sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
        batteryMsg.present = true;

        diagMsg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        diagMsg.name = "DHT Sensor";

        posMsg.header.frame_id = "map";
        posMsg.point.x = 0;
        posMsg.point.y = 0;
        posMsg.point.z = 0;
    }
};

}  // namespace dtnrobot

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dtnrobot::StatusPublisher>());
    rclcpp::shutdown();
    return 0;
}
