#ifndef NAV2_RRT_BSPLINE_SMOOTH_PLANNER_HPP_
#define NAV2_RRT_BSPLINE_SMOOTH_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace nav2_rrt_bspline_smooth_planner
{

// RRT树节点结构体
struct Node {
  double x, y;
  int parent_idx;
  Node(double nx, double ny, int p_idx) : x(nx), y(ny), parent_idx(p_idx) {}
};

class RRTBSplineSmoothPlanner : public nav2_core::GlobalPlanner
{
public:
  RRTBSplineSmoothPlanner() = default;
  ~RRTBSplineSmoothPlanner() = default;

  // Nav2 插件必须重写的核心生命周期函数
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  // 核心规划接口
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // 碰撞检测函数
  bool isLineClear(double x1, double y1, double x2, double y2);

  // 三次 B 样条平滑函数声明
  std::vector<geometry_msgs::msg::PoseStamped> bsplineSmooth(
    const std::vector<geometry_msgs::msg::PoseStamped> & control_points,
    int sample_num);

  // 基础成员变量
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;
  std::string global_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D * costmap_;

  // 用于在 RViz 中可视化 RRT 树的发布者
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

}  // namespace nav2_rrt_bspline_smooth_planner

#endif  // NAV2_RRT_BSPLINE_SMOOTH_PLANNER_HPP_