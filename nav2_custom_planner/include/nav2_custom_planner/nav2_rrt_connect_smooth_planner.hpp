#ifndef NAV2_CUSTOM_PLANNER__NAV2_RRT_CONNECT_SMOOTH_PLANNER_HPP_
#define NAV2_CUSTOM_PLANNER__NAV2_RRT_CONNECT_SMOOTH_PLANNER_HPP_

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace nav2_rrt_connect_smooth_planner
{

// RRT 树节点定义
struct Node {
  double x;
  double y;
  int parent_idx;
  Node(double _x, double _y, int _parent_idx) 
  : x(_x), y(_y), parent_idx(_parent_idx) {}
};

class RRTConnectSmoothPlanner : public nav2_core::GlobalPlanner
{
public:
  RRTConnectSmoothPlanner() = default;
  ~RRTConnectSmoothPlanner() = default;

  // -------------------------------------------------------------------------
  // nav2_core::GlobalPlanner 接口覆写
  // -------------------------------------------------------------------------
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
  // -------------------------------------------------------------------------
  // 自定义辅助函数
  // -------------------------------------------------------------------------
  
  // 碰撞检测：判断两点之间的直线路径是否没有障碍物
  bool isLineClear(double x1, double y1, double x2, double y2);

  // B样条平滑算法
  std::vector<geometry_msgs::msg::PoseStamped> bsplineSmooth(
    const std::vector<geometry_msgs::msg::PoseStamped> & control_points,
    int sample_num);

  // -------------------------------------------------------------------------
  // 成员变量
  // -------------------------------------------------------------------------
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;
  std::string global_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D * costmap_;
  
  // 用于在 RViz 中可视化 RRT 树的发布者
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

}  // namespace nav2_rrt_connect_smooth_planner

#endif  // NAV2_CUSTOM_PLANNER__NAV2_RRT_CONNECT_SMOOTH_PLANNER_HPP_