#include <chrono>
#include <functional>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace dtnrobot {

using namespace std::chrono_literals;

class ImagePublisher : public rclcpp::Node {
public:
    ImagePublisher() : Node("image_publisher") {
        imageSubscriber = image_transport::create_subscription(
            this, "/camera/image_raw",
            std::bind(&ImagePublisher::imageCallback, this, std::placeholders::_1), "raw");
        imagePublisher = image_transport::create_publisher(this, "detectedImages");

        dist = std::bernoulli_distribution(0.1);
        timer = this->create_wall_timer(10s, std::bind(&ImagePublisher::timer_callback, this));
    }

private:
    std::default_random_engine generator;
    std::bernoulli_distribution dist;

    rclcpp::TimerBase::SharedPtr timer;

    image_transport::Subscriber imageSubscriber;
    image_transport::Publisher imagePublisher;

    sensor_msgs::msg::Image::ConstSharedPtr lastImage;

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) { lastImage = msg; }

    void timer_callback() {
        if (lastImage && dist(generator)) {
            imagePublisher.publish(lastImage);
        }
    }
};

}  // namespace dtnrobot

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dtnrobot::ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
