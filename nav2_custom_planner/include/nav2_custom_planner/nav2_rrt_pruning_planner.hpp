#ifndef NAV2_RRT_PRUNING_PLANNER_HPP_
#define NAV2_RRT_PRUNING_PLANNER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace nav2_rrt_pruning_planner
{

struct Node {
  double x, y;
  int parent_idx;
  Node(double x_, double y_, int p_) : x(x_), y(y_), parent_idx(p_) {}
};

class RRTPruningPlanner : public nav2_core::GlobalPlanner
{
public:
  RRTPruningPlanner() = default;
  ~RRTPruningPlanner() = default;

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

private:
  bool isLineClear(double x1, double y1, double x2, double y2);
  double getDistanceToObstacle(double x, double y);

  nav2_costmap_2d::Costmap2D * costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string global_frame_, name_;
  
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

}  // namespace nav2_rrt_pruning_planner

#endif