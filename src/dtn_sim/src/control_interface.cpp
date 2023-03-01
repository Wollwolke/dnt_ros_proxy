#include <functional>
#include <future>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"

namespace dtnsim {

class ControlInterface : public rclcpp::Node {
public:
    using FollowWp = nav2_msgs::action::FollowWaypoints;
    using GoalHandleFollowWp = rclcpp_action::ClientGoalHandle<FollowWp>;

    explicit ControlInterface() : Node("control_interface") {
        this->client_ptr_ = rclcpp_action::create_client<FollowWp>(this, "follow_waypoints");

        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                               std::bind(&ControlInterface::send_goal, this));
        RCLCPP_INFO(get_logger(), "Wolke");
    }

    void send_goal() {
        using namespace std::placeholders;

        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = FollowWp::Goal();
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;

        geometry_msgs::msg::PoseStamped wp;
        wp.header.frame_id = "map";
        wp.header.stamp = now();

        for (auto n : {0, 1, 2}) {
            wp.pose.position.x = 1;
            wp.pose.position.y = n;
            waypoints.push_back(wp);
        }

        goal_msg.poses = waypoints;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<FollowWp>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&ControlInterface::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&ControlInterface::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&ControlInterface::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<FollowWp>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    void goal_response_callback(const GoalHandleFollowWp::SharedPtr& goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(GoalHandleFollowWp::SharedPtr,
                           const std::shared_ptr<const FollowWp::Feedback> feedback) {
        // std::stringstream ss;
        // ss << "Next number in sequence received: ";
        // for (auto number : feedback->partial_sequence) {
        //     ss << number << " ";
        // }
        // RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

    void result_callback(const GoalHandleFollowWp::WrappedResult& result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
        // std::stringstream ss;
        // ss << "Result received: ";
        // for (auto number : result.result->sequence) {
        //     ss << number << " ";
        // }
        // RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        rclcpp::shutdown();
    }
};

}  // namespace dtnsim

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dtnsim::ControlInterface>());
    rclcpp::shutdown();
    return 0;
}
