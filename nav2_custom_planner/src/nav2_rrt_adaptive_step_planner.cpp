#include "nav2_custom_planner/nav2_rrt_adaptive_step_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <random>
#include <algorithm>
#include <chrono>
#include "pluginlib/class_list_macros.hpp"

namespace nav2_rrt_adaptive_step_planner
{

void RRTAdaptiveStepPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("rrt_tree_markers", 10);
  RCLCPP_INFO(node_->get_logger(), "自适应步长 RRT 规划器已配置。增加连线碰撞检测以防止穿墙。");
}

void RRTAdaptiveStepPlanner::cleanup() { marker_pub_.reset(); }
void RRTAdaptiveStepPlanner::activate() { marker_pub_->on_activate(); }
void RRTAdaptiveStepPlanner::deactivate() { marker_pub_->on_deactivate(); }

double RRTAdaptiveStepPlanner::getDistanceToObstacle(double x, double y)
{
  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my)) return 0.0;

  double min_dist = 1.0; 
  int cell_radius = static_cast<int>(min_dist / costmap_->getResolution());

  for (int i = -cell_radius; i <= cell_radius; ++i) {
    for (int j = -cell_radius; j <= cell_radius; ++j) {
      unsigned int cx = mx + i;
      unsigned int cy = my + j;
      if (cx < costmap_->getSizeInCellsX() && cy < costmap_->getSizeInCellsY()) {
        unsigned char cost = costmap_->getCost(cx, cy);
        if (cost >= 253) { // Lethal obstacle
          double wx, wy;
          costmap_->mapToWorld(cx, cy, wx, wy);
          double d = std::hypot(x - wx, y - wy);
          if (d < min_dist) min_dist = d;
        }
      }
    }
  }
  return min_dist;
}

nav_msgs::msg::Path RRTAdaptiveStepPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  auto start_time_clock = std::chrono::steady_clock::now();
  auto now = node_->now(); 

  nav_msgs::msg::Path global_path;
  global_path.header.stamp = now;
  global_path.header.frame_id = global_frame_;

  visualization_msgs::msg::Marker tree_marker;
  tree_marker.header.stamp = now;
  tree_marker.header.frame_id = global_frame_;
  tree_marker.ns = "adaptive_rrt";
  tree_marker.id = 0;
  tree_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  tree_marker.action = visualization_msgs::msg::Marker::ADD;
  tree_marker.scale.x = 0.02;
  tree_marker.color.r = 0.0; tree_marker.color.g = 0.8; tree_marker.color.b = 1.0; tree_marker.color.a = 0.8;
  tree_marker.pose.orientation.w = 1.0;

  // --- 算法参数优化 ---
  const double s_min = 0.15;    
  const double s_max = 0.45;    
  const double d_safe = 0.6;    
  const int max_iter = 3000; 
  const double goal_tol = 0.35;
  const double eta = 0.5, zeta = 0.2, d0 = 0.8; // 加大 APF 斥力权重和范围，让路径更居中

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis_x(costmap_->getOriginX(), costmap_->getOriginX() + costmap_->getSizeInMetersX());
  std::uniform_real_distribution<> dis_y(costmap_->getOriginY(), costmap_->getOriginY() + costmap_->getSizeInMetersY());
  std::uniform_real_distribution<> dis_prob(0.0, 1.0);

  std::vector<Node> tree;
  tree.emplace_back(start.pose.position.x, start.pose.position.y, -1);

  bool found = false;
  for (int i = 0; i < max_iter; ++i) {
    double ratio = static_cast<double>(i) / max_iter;
    double p_bias = (ratio < 0.3) ? 0.1 + 0.3 * std::log2(1.0 + ratio) : 0.45;
    double rx = (dis_prob(gen) < p_bias) ? goal.pose.position.x : dis_x(gen);
    double ry = (dis_prob(gen) < p_bias) ? goal.pose.position.y : dis_y(gen);

    int n_idx = 0;
    double m_dist = 1e9;
    for (size_t j = 0; j < tree.size(); ++j) {
      double d = std::hypot(tree[j].x - rx, tree[j].y - ry);
      if (d < m_dist) { m_dist = d; n_idx = j; }
    }

    double d_real = getDistanceToObstacle(tree[n_idx].x, tree[n_idx].y);
    double current_step = (d_real >= d_safe) ? s_max : (s_min + (s_max - s_min) * (d_real / d_safe));

    double angle = std::atan2(ry - tree[n_idx].y, rx - tree[n_idx].x);
    double nx = tree[n_idx].x + current_step * std::cos(angle);
    double ny = tree[n_idx].y + current_step * std::sin(angle);

    // APF 修正
    double fx = zeta * (goal.pose.position.x - nx);
    double fy = zeta * (goal.pose.position.y - ny);
    unsigned int mx, my;
    if (costmap_->worldToMap(nx, ny, mx, my)) {
      int r = 3; 
      for (int ox = -r; ox <= r; ox++) {
        for (int oy = -r; oy <= r; oy++) {
          unsigned int cx = mx + ox, cy = my + oy;
          if (cx < costmap_->getSizeInCellsX() && cy < costmap_->getSizeInCellsY()) {
            if (costmap_->getCost(cx, cy) >= 200) { // 使用更严格的 200 (膨胀层) 增加安全余量
              double wx, wy; costmap_->mapToWorld(cx, cy, wx, wy);
              double d = std::hypot(nx - wx, ny - wy);
              if (d < d0 && d > 0.01) {
                double rep = eta * (1.0/d - 1.0/d0) * (1.0/(d*d));
                fx += rep * (nx - wx) / d;
                fy += rep * (ny - wy) / d;
              }
            }
          }
        }
      }
    }
    nx += 0.1 * fx; ny += 0.1 * fy;

    // --- 核心改进：连线碰撞检测 ---
    bool collision = false;
    double check_dist = std::hypot(nx - tree[n_idx].x, ny - tree[n_idx].y);
    int steps = static_cast<int>(check_dist / costmap_->getResolution());
    for (int j = 1; j <= steps; ++j) {
      double interp_x = tree[n_idx].x + (nx - tree[n_idx].x) * (static_cast<double>(j) / steps);
      double interp_y = tree[n_idx].y + (ny - tree[n_idx].y) * (static_cast<double>(j) / steps);
      unsigned int cmx, cmy;
      if (!costmap_->worldToMap(interp_x, interp_y, cmx, cmy) || costmap_->getCost(cmx, cmy) >= 253) {
        collision = true;
        break;
      }
    }
    if (collision) continue;

    tree.emplace_back(nx, ny, n_idx);
    
    geometry_msgs::msg::Point p1, p2;
    p1.x = tree[n_idx].x; p1.y = tree[n_idx].y; p1.z = 0.15;
    p2.x = nx; p2.y = ny; p2.z = 0.15;
    tree_marker.points.push_back(p1); tree_marker.points.push_back(p2);

    if (std::hypot(nx - goal.pose.position.x, ny - goal.pose.position.y) < goal_tol) {
      found = true; break;
    }
  }

  marker_pub_->publish(tree_marker);

  auto end_time_clock = std::chrono::steady_clock::now();
  double duration = std::chrono::duration<double, std::milli>(end_time_clock - start_time_clock).count();

  if (found) {
    int curr = tree.size() - 1;
    while (curr != -1) {
      geometry_msgs::msg::PoseStamped p;
      p.header = global_path.header;
      p.pose.position.x = tree[curr].x; p.pose.position.y = tree[curr].y;
      p.pose.orientation.w = 1.0;
      global_path.poses.push_back(p);
      curr = tree[curr].parent_idx;
    }
    std::reverse(global_path.poses.begin(), global_path.poses.end());

    RCLCPP_INFO(node_->get_logger(), 
      "RRT 规划结束 | 采样节点数: %ld | 耗时: %.2f ms | 结果: 成功", 
      tree.size(), duration);
    return global_path;
  }

  RCLCPP_WARN(node_->get_logger(), "RRT 规划失败 | 耗时: %.2f ms", duration);
  return global_path;
}

} // namespace nav2_rrt_adaptive_step_planner

PLUGINLIB_EXPORT_CLASS(nav2_rrt_adaptive_step_planner::RRTAdaptiveStepPlanner, nav2_core::GlobalPlanner)