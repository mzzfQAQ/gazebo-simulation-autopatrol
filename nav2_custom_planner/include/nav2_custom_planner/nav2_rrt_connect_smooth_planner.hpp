#ifndef NAV2_RRT_CONNECT_SMOOTH_PLANNER__NAV2_RRT_CONNECT_SMOOTH_PLANNER_HPP_
#define NAV2_RRT_CONNECT_SMOOTH_PLANNER__NAV2_RRT_CONNECT_SMOOTH_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <random>
#include <algorithm>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_rrt_connect_smooth_planner
{

struct Node {
  double x, y;
  int parent_idx;
};

class RRTConnectSmoothPlanner : public nav2_core::GlobalPlanner
{
public:
  RRTConnectSmoothPlanner() = default;
  ~RRTConnectSmoothPlanner() override = default;

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
  // 核心算法函数（保持你的逻辑）
  bool extendTree(std::vector<Node> & tree, const Node & target, Node & new_node);
  double getDistance(const Node & n1, const Node & n2);
  bool isCollisionFree(const Node & n1, const Node & n2);
  
  // 新增：后处理函数
  void optimizePath(nav_msgs::msg::Path & path);
  void smoothPath(nav_msgs::msg::Path & path, double target_spacing);
  
  void publishTree(
    const std::vector<Node> & start_tree, 
    const std::vector<Node> & goal_tree, 
    const std::string & frame_id);

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("RRTConnectSmooth")};

  // 参数默认值
  double step_size_{0.2};
  double connection_threshold_{0.5};
  int max_iterations_{5000};
  double interpolation_resolution_{0.05};
};

}  // namespace nav2_rrt_connect_smooth_planner

#endif