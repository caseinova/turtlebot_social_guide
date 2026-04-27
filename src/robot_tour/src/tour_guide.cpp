#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "nav2_msgs/action/follow_waypoints.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"

namespace robot_tour
{
class WaypointFollowerClient : public rclcpp::Node
{
public:
  using Waypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleWaypoints = rclcpp_action::ClientGoalHandle<Waypoints>;

  explicit WaypointFollowerClient(const rclcpp::NodeOptions & options)
  : Node("waypoint_action_client", options)
  {
    geometry_msgs::msg::PoseStamped x;
    x.header.frame_id = "map";
    x.pose.position.x=1.3623204231262207;
    x.pose.position.y=-1.4709776639938354;
    this->poses_.push_back(x);
    x.pose.position.x=-0.6929791569709778;
    x.pose.position.y=1.9281070232391357;
    this->poses_.push_back(x);
    this->client_ptr_ = rclcpp_action::create_client<Waypoints>(
      this,
      "/follow_waypoints");
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "tour_command", 10, std::bind(&WaypointFollowerClient::topic_callback, this, std::placeholders::_1));
    
    // this->timer_ = this->create_wall_timer(
    //   std::chrono::milliseconds(500),
    //   std::bind(&WaypointFollowerClient::send_goal, this, _1));
  }

  void send_goal(std::vector<geometry_msgs::msg::PoseStamped> poses)
  {
    using namespace std::placeholders;

    // this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Waypoints::Goal();
    goal_msg.poses = poses;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Waypoints>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&WaypointFollowerClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&WaypointFollowerClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&WaypointFollowerClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp_action::Client<Waypoints>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<geometry_msgs::msg::PoseStamped> poses_;

  void goal_response_callback(std::shared_ptr<GoalHandleWaypoints> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleWaypoints::SharedPtr,
    const std::shared_ptr<const Waypoints::Feedback> feedback)
  {
    // std::stringstream ss;
    // ss << "Next number in sequence received: ";
    // for (auto number : feedback->partial_sequence) {
    //   ss << number << " ";
    // }
    RCLCPP_INFO(this->get_logger(), "The current goal is %d", feedback->current_waypoint);
  }

  void result_callback(const GoalHandleWaypoints::WrappedResult & result)
  {
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
    //   ss << number << " ";
    // }
    for (long unsigned int i=0;i<size(result.result->missed_waypoints);i++)
    {
    RCLCPP_INFO(this->get_logger(), "Missed %u \n", result.result->missed_waypoints[i]);
    }
    rclcpp::shutdown();
  }
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "received %s", msg->data.c_str());
    this->send_goal(this->poses_);
    RCLCPP_INFO(this->get_logger(), "goal sent");
  }
};  // class FibonacciActionClient

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(robot_tour::WaypointFollowerClient)