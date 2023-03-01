#include <chrono>
#include <functional>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <vector>

#include "dtn_robot/msg/point_array.hpp"
#include "dtn_robot/srv/detail/follow_waypoints__struct.hpp"
#include "dtn_robot/srv/follow_waypoints.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"

namespace dtnrobot {

using namespace std::placeholders;

class ControlInterface : public rclcpp::Node {
public:
    using FollowWp = nav2_msgs::action::FollowWaypoints;
    using GoalHandleFollowWp = rclcpp_action::ClientGoalHandle<FollowWp>;

    explicit ControlInterface() : Node("control_interface") {
        this->client = rclcpp_action::create_client<FollowWp>(this, "follow_waypoints");
        this->service = this->create_service<dtn_robot::srv::FollowWaypoints>(
            "follow_waypoints", std::bind(&ControlInterface::serviceCallback, this, _1, _2));

        RCLCPP_INFO(get_logger(), "Control interface up.");
    }

private:
    using Request_t = std::shared_ptr<dtn_robot::srv::FollowWaypoints::Request>;
    using Response_t = std::shared_ptr<dtn_robot::srv::FollowWaypoints::Response>;

    rclcpp_action::Client<FollowWp>::SharedPtr client;
    rclcpp::Service<dtn_robot::srv::FollowWaypoints>::SharedPtr service;
    bool gotGoalResponse = false;
    bool goalResponse;

    void sendGoal(std::vector<geometry_msgs::msg::PoseStamped> waypoints, Response_t response) {
        using namespace std::chrono_literals;

        if (!this->client->wait_for_action_server(1s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goalMsg = FollowWp::Goal();
        goalMsg.poses = waypoints;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto goalResponseCb = [this, response](const GoalHandleFollowWp::SharedPtr& goalHandle) {
            if (!goalHandle) {
                RCLCPP_INFO(this->get_logger(), "Goal was rejected by server");
                this->goalResponse = false;
            } else {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
                this->goalResponse = true;
            }
            this->gotGoalResponse = true;
        };

        auto send_goal_options = rclcpp_action::Client<FollowWp>::SendGoalOptions();
        send_goal_options.goal_response_callback = goalResponseCb;
        // std::bind(&ControlInterface::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&ControlInterface::feedbackCallback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&ControlInterface::resultCallback, this, _1);
        this->client->async_send_goal(goalMsg, send_goal_options);
    }

    void serviceCallback(const std::shared_ptr<dtn_robot::srv::FollowWaypoints::Request> request,
                         std::shared_ptr<dtn_robot::srv::FollowWaypoints::Response> response) {
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        geometry_msgs::msg::PoseStamped waypoint;

        for (auto point : request->waypoints.points) {
            waypoint.header = request->waypoints.header;
            waypoint.pose.position.x = point.x;
            waypoint.pose.position.y = point.y;
            waypoints.push_back(waypoint);
        }

        sendGoal(waypoints, response);

        // TODO: fix this:
        // Can't use future / conditionvariable because I can't block this thread
        response->request_accepted = goalResponse;
        gotGoalResponse = false;
    }

    void feedbackCallback(GoalHandleFollowWp::SharedPtr,
                          const std::shared_ptr<const FollowWp::Feedback> feedback) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Current Waypoint: " << feedback->current_waypoint);
    }

    void resultCallback(const GoalHandleFollowWp::WrappedResult& result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal was succeeded");
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
    }
};

}  // namespace dtnrobot

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dtnrobot::ControlInterface>());
    rclcpp::shutdown();
    return 0;
}
