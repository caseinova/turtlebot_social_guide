#include "robot_tour/subtour.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"

namespace robot_tour
{

SubtourNode::SubtourNode(const rclcpp::NodeOptions & options)
: Node("subtour_action_node", options)
{
  const auto command_topic = this->declare_parameter<std::string>("command_topic", "/tsp_command");
  const auto action_name = this->declare_parameter<std::string>("action_name", "/follow_waypoints");
  max_2opt_iterations_ = this->declare_parameter<int>("max_2opt_iterations", 1000);

  waypoint_client_ = rclcpp_action::create_client<Waypoints>(this, action_name);
  tsp_subscription_ = this->create_subscription<social_robot_interfaces::msg::TspCommand>(
    command_topic,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&SubtourNode::tspCommandCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "Listening for TSP waypoint lists on '%s' and sending tours to '%s'",
    command_topic.c_str(),
    action_name.c_str());
}

float SubtourNode::computeCost(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b)
{
  const auto dx = a.pose.position.x - b.pose.position.x;
  const auto dy = a.pose.position.y - b.pose.position.y;
  return static_cast<float>(std::hypot(dx, dy));
}

int SubtourNode::getClosestNodeIdx(int current_node_idx, const std::vector<int> & unvisited_nodes)
{
  auto min_cost = large_number;
  auto min_idx = -1;

  for (const auto node_idx : unvisited_nodes) {
    const auto cost = cost_matrix.at(current_node_idx).at(node_idx);
    if (cost < min_cost) {
      min_cost = cost;
      min_idx = node_idx;
    }
  }

  return min_idx;
}

bool SubtourNode::makeCostMatrix(const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  num_nodes = static_cast<int>(poses.size());
  cost_matrix.assign(num_nodes, std::vector<float>(num_nodes, 0.0F));

  for (int i = 0; i < num_nodes; ++i) {
    for (int j = 0; j < num_nodes; ++j) {
      cost_matrix[i][j] = computeCost(poses[i], poses[j]);
    }
  }

  return true;
}

float SubtourNode::getCost(int x, int y)
{
  return cost_matrix.at(x).at(y);
}

bool SubtourNode::initializeTour()
{
  current_tour.clear();

  if (num_nodes <= 0) {
    return false;
  }

  std::vector<int> unvisited_nodes(num_nodes - 1);
  std::iota(unvisited_nodes.begin(), unvisited_nodes.end(), 1);

  current_tour.push_back(0);
  auto current_node = 0;

  while (!unvisited_nodes.empty()) {
    const auto next_node = getClosestNodeIdx(current_node, unvisited_nodes);
    if (next_node < 0) {
      return false;
    }

    current_tour.push_back(next_node);
    unvisited_nodes.erase(
      std::remove(unvisited_nodes.begin(), unvisited_nodes.end(), next_node),
      unvisited_nodes.end());
    current_node = next_node;
  }

  return true;
}

float SubtourNode::computeTourCost()
{
  if (current_tour.size() < 2) {
    return 0.0F;
  }

  auto total_cost = 0.0F;
  for (std::size_t i = 1; i < current_tour.size(); ++i) {
    total_cost += getCost(current_tour[i - 1], current_tour[i]);
  }

  return total_cost;
}

bool SubtourNode::improveTour(int max_iterations)
{
  if (current_tour.size() < 4) {
    return true;
  }

  auto iterations = 0;
  auto improved = true;

  while (improved && iterations < max_iterations) {
    improved = false;

    for (std::size_t i = 1; i + 2 < current_tour.size() && iterations < max_iterations; ++i) {
      for (std::size_t k = i + 1; k + 1 < current_tour.size() && iterations < max_iterations; ++k) {
        const auto a = current_tour[i - 1];
        const auto b = current_tour[i];
        const auto c = current_tour[k];
        const auto d = current_tour[k + 1];
        const auto old_cost = getCost(a, b) + getCost(c, d);
        const auto new_cost = getCost(a, c) + getCost(b, d);

        ++iterations;
        if (new_cost + std::numeric_limits<float>::epsilon() < old_cost) {
          std::reverse(current_tour.begin() + static_cast<long>(i), current_tour.begin() + static_cast<long>(k + 1));
          improved = true;
        }
      }
    }
  }

  return true;
}

std::vector<geometry_msgs::msg::PoseStamped> SubtourNode::solveTour(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses,
  int max_iterations)
{
  makeCostMatrix(poses);
  initializeTour();
  improveTour(max_iterations);

  std::vector<geometry_msgs::msg::PoseStamped> ordered_poses;
  ordered_poses.reserve(current_tour.size());
  for (const auto pose_idx : current_tour) {
    ordered_poses.push_back(poses.at(pose_idx));
  }

  return ordered_poses;
}

void SubtourNode::tspCommandCallback(const social_robot_interfaces::msg::TspCommand::SharedPtr msg)
{
  if (msg->poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received an empty TSP command; ignoring it");
    return;
  }

  auto ordered_poses = solveTour(msg->poses, max_2opt_iterations_);

  RCLCPP_INFO(
    this->get_logger(),
    "Optimized %zu waypoints; final path length is %.3f m",
    ordered_poses.size(),
    computeTourCost());

  sendGoal(ordered_poses);
}

void SubtourNode::sendGoal(const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  using namespace std::chrono_literals;

  if (!waypoint_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "Waypoint follower action server is not available");
    return;
  }

  auto goal_msg = Waypoints::Goal();
  goal_msg.poses = poses;

  auto send_goal_options = rclcpp_action::Client<Waypoints>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&SubtourNode::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&SubtourNode::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&SubtourNode::resultCallback, this, std::placeholders::_1);

  waypoint_client_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(this->get_logger(), "Sent %zu optimized waypoints", poses.size());
}

void SubtourNode::goalResponseCallback(const GoalHandleWaypoints::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Waypoint follower rejected the goal");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Waypoint follower accepted the goal");
}

void SubtourNode::feedbackCallback(
  GoalHandleWaypoints::SharedPtr,
  const std::shared_ptr<const Waypoints::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Current waypoint: %u", feedback->current_waypoint);
}

void SubtourNode::resultCallback(const GoalHandleWaypoints::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(
        this->get_logger(),
        "Waypoint tour completed with %zu missed waypoints",
        result.result->missed_waypoints.size());
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Waypoint tour was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "Waypoint tour was canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Waypoint tour returned an unknown result code");
      break;
  }
}

}  // namespace robot_tour

RCLCPP_COMPONENTS_REGISTER_NODE(robot_tour::SubtourNode)
