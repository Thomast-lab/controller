#ifndef NAV2_THOMAS_PURSUIT__THOMAS_PURSUIT_HPP_
#define NAV2_THOMAS_PURSUIT__THOMAS_PURSUIT_HPP_

#include <string>
#include <vector>
#include <memory>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_thomas_pursuit_
{

class ThomasPursuit : public nav2_core::Controller
{
public:
  ThomasPursuit() = default;
  ~ThomasPursuit() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr< ::nav2_costmap_2d::Costmap2DROS > costmap_ros
  ) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

protected:
  // Transforme le plan global dans le repère du robot
  nav_msgs::msg::Path transformGlobalPlan(const nav_msgs::msg::Path & path);

  // membres ros et configuration
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr< ::nav2_costmap_2d::Costmap2DROS > costmap_ros_;
  rclcpp::Logger logger_ {rclcpp::get_logger("ThomasPursuit")};
  rclcpp::Clock::SharedPtr clock_;

  // params du contrôleur
  double desired_linear_vel_;
  double lookahead_dist_;
  double max_angular_vel_;
  rclcpp::Duration transform_tolerance_ {0, 0};
  double min_lookahead_dist_;
  double max_lookahead_dist_;
  double goal_tolerance_;
  double curvature_limit_;

  // plan global transformé
  nav_msgs::msg::Path global_plan_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr global_pub_;
};

} // namespace nav2_thomas_pursuit_

#endif