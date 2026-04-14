#include "stanley_controller/stanley_controller.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/goal_checker.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>
#include <chrono>
#include <limits>

namespace stanley_controller
{

void StanleyController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  // Declare and get parameters
  // Defaults suitable for QCar
  node->declare_parameter(plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
  node->declare_parameter(plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));
  node->declare_parameter(plugin_name_ + ".k_gain", rclcpp::ParameterValue(0.5)); 
  node->declare_parameter(plugin_name_ + ".wheelbase", rclcpp::ParameterValue(0.256)); 

  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(plugin_name_ + ".k_gain", k_gain_);
  node->get_parameter(plugin_name_ + ".wheelbase", wheelbase_);

  RCLCPP_INFO(node->get_logger(), 
    "Stanley Controller configured: desired_vel=%.2f, k_gain=%.2f, wheelbase=%.3f",
    desired_linear_vel_, k_gain_, wheelbase_);
}

void StanleyController::activate() 
{
  RCLCPP_INFO(rclcpp::get_logger("StanleyController"), "Activating controller");
}

void StanleyController::deactivate() 
{
  RCLCPP_INFO(rclcpp::get_logger("StanleyController"), "Deactivating controller");
}

void StanleyController::cleanup() 
{
  RCLCPP_INFO(rclcpp::get_logger("StanleyController"), "Cleaning up controller");
}

void StanleyController::setPlan(const nav_msgs::msg::Path & path) 
{
  global_plan_ = path;
  last_closest_idx_ = 0;
}

void StanleyController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  (void)percentage;
  if (speed_limit > 0.0) {
    desired_linear_vel_ = speed_limit;
  }
}

geometry_msgs::msg::TwistStamped StanleyController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = "base_link";
  cmd_vel.header.stamp = pose.header.stamp;

  // 1. Transform global plan to robot frame (base_link)
  auto transformed_plan = transformGlobalPlan(pose);
  
  if (transformed_plan.poses.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("StanleyController"), 
      "Transformed plan is empty, stopping robot");
    return cmd_vel;
  }

  // 2. Project Position to FRONT AXLE
  // In base_link (rear axle), front axle is at (wheelbase, 0)
  double front_x = wheelbase_;
  double front_y = 0.0; 

  // 3. Find closest point on path to the FRONT AXLE
  // We search the transformed plan, which is already a window around the robot
  size_t closest_index = 0;
  double min_dist_sq = std::numeric_limits<double>::max();

  for (size_t i = 0; i < transformed_plan.poses.size(); ++i) {
    double px = transformed_plan.poses[i].pose.position.x;
    double py = transformed_plan.poses[i].pose.position.y;
    
    double dist_sq = std::pow(px - front_x, 2) + std::pow(py - front_y, 2);
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_index = i;
    }
  }

  // 4. Calculate Cross Track Error (e)
  double cx = transformed_plan.poses[closest_index].pose.position.x;
  double cy = transformed_plan.poses[closest_index].pose.position.y;
  
  // In robot frame, positive y is left. So e = cy.
  double error_cross_track = cy; 

  // 5. Calculate Heading Error (psi)
  double path_heading = 0.0;
  
  if (closest_index + 1 < transformed_plan.poses.size()) {
    double next_x = transformed_plan.poses[closest_index + 1].pose.position.x;
    double next_y = transformed_plan.poses[closest_index + 1].pose.position.y;
    path_heading = std::atan2(next_y - cy, next_x - cx);
  } else if (closest_index > 0) {
    double prev_x = transformed_plan.poses[closest_index - 1].pose.position.x;
    double prev_y = transformed_plan.poses[closest_index - 1].pose.position.y;
    path_heading = std::atan2(cy - prev_y, cx - prev_x);
  }

  double error_heading = path_heading;

  // Normalize angle to [-pi, pi]
  while (error_heading > M_PI) error_heading -= 2.0 * M_PI;
  while (error_heading < -M_PI) error_heading += 2.0 * M_PI;

  // 6. Compute Steering Angle (Stanley Law)
  double current_speed = velocity.linear.x;
  double v_denominator = std::max(std::abs(current_speed), 0.1); 

  double delta = error_heading + std::atan2(k_gain_ * error_cross_track, v_denominator);

  // 7. Clamp steering
  delta = std::max(std::min(delta, max_angular_vel_), -max_angular_vel_);

  // 8. Convert Steering Angle (delta) to Angular Velocity (omega)
  double angular_vel = (desired_linear_vel_ / wheelbase_) * std::tan(delta);

  cmd_vel.twist.linear.x = desired_linear_vel_;
  cmd_vel.twist.angular.z = angular_vel;

  return cmd_vel;
}

nav_msgs::msg::Path StanleyController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  nav_msgs::msg::Path local_path;
  local_path.header.frame_id = "base_link";
  local_path.header.stamp = pose.header.stamp;

  if (global_plan_.poses.empty()) {
    return local_path;
  }

  // Find closest point to robot in global frame to avoid transforming the whole path
  // Search forward from last known position (don't search backwards or across the track)
  size_t search_start = (last_closest_idx_ > 10) ? last_closest_idx_ - 10 : 0;
  size_t closest_idx = search_start;
  double min_dist_sq = std::numeric_limits<double>::max();
  for (size_t i = search_start; i < std::min(global_plan_.poses.size(), search_start + 200); ++i) {
    double dx = global_plan_.poses[i].pose.position.x - pose.pose.position.x;
    double dy = global_plan_.poses[i].pose.position.y - pose.pose.position.y;
    double d2 = dx*dx + dy*dy;
    if (d2 < min_dist_sq) {
      min_dist_sq = d2;
      closest_idx = i;
    }
  }
  last_closest_idx_ = closest_idx;

  // Transform a window of points (e.g., 50 points forward)
  size_t window_size = 50;
  for (size_t i = closest_idx; i < std::min(global_plan_.poses.size(), closest_idx + window_size); ++i) {
    try {
      geometry_msgs::msg::PoseStamped local_pose;
      tf_->transform(global_plan_.poses[i], local_pose, "base_link",
                     tf2::durationFromSec(0.1));
      local_path.poses.push_back(local_pose);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(rclcpp::get_logger("StanleyController"), 
        "TF Transform failed at index %zu: %s", i, ex.what());
    }
  }

  return local_path;
}

}  // namespace stanley_controller

// Register this controller as a nav2_core::Controller plugin
PLUGINLIB_EXPORT_CLASS(stanley_controller::StanleyController, nav2_core::Controller)
