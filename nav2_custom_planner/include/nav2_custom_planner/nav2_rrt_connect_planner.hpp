#ifndef NAV2_RRT_CONNECT_PLANNER__RRT_CONNECT_PLANNER_HPP_
#define NAV2_RRT_CONNECT_PLANNER__RRT_CONNECT_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <random>
#include <algorithm>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace nav2_rrt_connect_planner
{

struct Node {
  double x, y;
  int parent_idx;
};

class RRTConnectPlanner : public nav2_core::GlobalPlanner
{
public:
  RRTConnectPlanner() = default;
  ~RRTConnectPlanner() override = default;

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
  // 核心辅助函数
  bool extendTree(std::vector<Node> & tree, const Node & target, Node & new_node);
  double getDistance(const Node & n1, const Node & n2);
  bool isCollisionFree(const Node & n1, const Node & n2);
  void publishTree(const std::vector<Node> & start_tree, const std::vector<Node> & goal_tree, const std::string & frame_id);
  
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  std::string name_;

  // RRT 参数
  double step_size_{0.2};
  int max_iterations_{5000};
  double connection_threshold_{0.3};
};

}  // namespace nav2_rrt_connect_planner

#endif