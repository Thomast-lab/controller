#include "thomas_pursuit/thomas_pursuit.hpp"
#include <algorithm>
#include <string>
#include <memory>
#include <vector>
#include <utility>
#include <limits>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

using rcl_interfaces::msg::ParameterType;
using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace nav2_thomas_pursuit_
{
void ThomasPursuit::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr< ::nav2_costmap_2d::Costmap2DROS > costmap_ros)

{
    node_ = parent ;
    auto node = node_.lock();

    costmap_ros_ = costmap_ros;
    tf_ = tf;
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    declare_parameter_if_not_declared(
        node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.6));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.3));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(0.9));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".goal_tolerance", rclcpp::ParameterValue(0.2));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".curvature_limit", rclcpp::ParameterValue(2.0));
    
    node-> get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
    node-> get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
    node-> get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
    double transform_tolerance;
    node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
    transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);
    node-> get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
    node-> get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
    node-> get_parameter(plugin_name_+ ".goal_tolerance", goal_tolerance);
    node-> get_parameter(plugin_name_ + ".curvature_limit", curvature_limit_);

    global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
} 

void ThomasPursuit::cleanup()
{
    RCLCPP_INFO(logger_, "Cleanup controller: %s of type nav2_thomas_pursuit_::Thomaspursuit",
    plugin_name_.c_str());
    global_pub_.reset();
}

void ThomasPursuit::activate()
{
    RCLCPP_INFO(logger_, "Activating controller: %s of type nav2_thomas_pursuit_::ThomasPursuit",
    plugin_name_.c_str());
    global_pub_->on_activate();
}

void ThomasPursuit::deactivate()
{
    RCLCPP_INFO(logger_, "Deactivating controller: %s of type nav2_thomas_pursuit_::ThomasPursuit",
    plugin_name_.c_str());
    global_pub_->on_deactivate();
}

void ThomasPursuit::setSpeedLimit(const double &speed_limit, const bool &percentage)
{
    (void) speed_limit;
    (void) percentage;
}

geometry_msgs::msg::TwistStamped ThomasPursuit::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker)
{
    (void) velocity;
    (void) goal_checker;
    // find the firste pose which is at a distance greater than the specified lkhd dist
    double effective_lookahead = std::clamp(lookahead_dist_, min_lookahead_dist_, max_lookahead_dist_);
    auto goal_pose = std::find_if(
        global_plan_.poses.begin(), global_plan_.poses.end(),
        [&](const auto & global_plan_pose) {
            return hypot(
                global_plan_pose.pose.position.x, global_plan_pose.pose.position.y) >= effective_lookahead;
        })->pose;

    double linear_vel, angular_vel;

// If the goal pose is in front of the robot then compute the velocity using the pure pursuit algorithm
// else rotate with the max angular velocity until the goal pose is in front of the robot

    if (goal_pose.position.x > 0) {
    
        auto curvature = 2.0 * goal_pose.position.y /
            (goal_pose.position.x * goal_pose.position.x + goal_pose.position.y * goal_pose.position.y);
        curvature = std::clamp(curvature, -curvature_limit_, curvature_limit_);
        linear_vel = desired_linear_vel_ * curvature;
    } else {
    linear_vel = 0.0;
    angular_vel = max_angular_vel_;
    }

  // Verify if the robot has reached goal  
    auto last_pose = global_plan_.poses.back().pose;
    double dist_to_goal = hypot(last_pose.position.x - pose.pose.position.x,
                                last_pose.position.y - pose.pose.position.y);

    if (dist_to_goal <= goal_tolerance) {
        geometry_msgs::msg::TwistStamped stop_cmd;
        stop_cmd.header.frame_id = pose.header.frame_id;
        stop_cmd.header.stamp = clock_->now();
        stop_cmd.twist.linear.x = 0.0;
        stop_cmd.twist.angular.z = 0.0;
        return stop_cmd;
    }

  // Create and publish a TwistStamped message with the desired velocity
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.twist.linear.x = linear_vel;
    cmd_vel.twist.angular.z = max(
        -1.0 * abs(max_angular_vel_), min(
            angular_vel, abs(max_angular_vel_)));

    return cmd_vel;
}

nav_msgs::msg::Path ThomasPursuit::transformGlobalPlan(const nav_msgs::msg::Path & path)
{   
    nav_msgs::msg::Path transformed_path;
    transformed_path.header.frame_id = "base_link";
    transformed_path.header.stamp = path.header.stamp;

    for (const auto & pose : path.poses) {
        geometry_msgs::msg::PoseStamped transformed_pose;
        tf_->transform(pose, transformed_pose, "base_link");
        transformed_path.poses.push_back(transformed_pose);
    } 
    return transformed_path;
}

void ThomasPursuit::setPlan(const nav_msgs::msg::Path &path)
{
    // transform global path int othe robot's frame
    global_plan_ = transformGlobalPlan(path);
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_thomas_pursuit_::ThomasPursuit, nav2_core::Controller)