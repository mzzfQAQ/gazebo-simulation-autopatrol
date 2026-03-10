#ifndef NAV2_CUSTOM_PLANNER__NAV2_RRT_DYNAMIC_BIASED_PLANNER_HPP_
#define NAV2_CUSTOM_PLANNER__NAV2_RRT_DYNAMIC_BIASED_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace nav2_rrt_dynamic_biased_planner
{
class RRTDynamicBiasedPlanner : public nav2_core::GlobalPlanner
{
public:
  RRTDynamicBiasedPlanner() = default;
  ~RRTDynamicBiasedPlanner() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_util::LifecycleNode::SharedPtr node_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string global_frame_, name_;
  double interpolation_resolution_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};
}  // namespace nav2_rrt_dynamic_biased_planner

#endif  // NAV2_CUSTOM_PLANNER__NAV2_RRT_DYNAMIC_BIASED_PLANNER_HPP_