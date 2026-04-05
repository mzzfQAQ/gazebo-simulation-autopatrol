#ifndef NAV2_CUSTOM_PLANNER__NAV2_RRT_BSPLINE_SMOOTH_PLANNER_HPP_
#define NAV2_CUSTOM_PLANNER__NAV2_RRT_BSPLINE_SMOOTH_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_rrt_bspline_smooth_planner
{

struct Node {
  double x, y;
  int parent_idx;
  Node(double x, double y, int parent) : x(x), y(y), parent_idx(parent) {}
};

class RRTBSplineSmoothPlanner : public nav2_core::GlobalPlanner
{
public:
  RRTBSplineSmoothPlanner() = default;
  ~RRTBSplineSmoothPlanner() = default;

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
  bool isLineClear(double x1, double y1, double x2, double y2);
  
  std::vector<geometry_msgs::msg::PoseStamped> bsplineSmooth(
    const std::vector<geometry_msgs::msg::PoseStamped> & control_points,
    int sample_num);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string name_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string global_frame_;
  
  // 使用 LifecyclePublisher 以支持 on_activate
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>> marker_pub_;
};

}  // namespace nav2_rrt_bspline_smooth_planner

#endif  // NAV2_CUSTOM_PLANNER__NAV2_RRT_BSPLINE_SMOOTH_PLANNER_HPP_