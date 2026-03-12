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
  RCLCPP_INFO(node_->get_logger(), "轻量化自适应步长 RRT 已配置 (Cost-Based Step).");
}

void RRTAdaptiveStepPlanner::cleanup() { marker_pub_.reset(); }
void RRTAdaptiveStepPlanner::activate() { marker_pub_->on_activate(); }
void RRTAdaptiveStepPlanner::deactivate() { marker_pub_->on_deactivate(); }

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
  tree_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  tree_marker.action = visualization_msgs::msg::Marker::ADD;
  tree_marker.scale.x = 0.015;
  tree_marker.color.r = 0.1; tree_marker.color.g = 0.9; tree_marker.color.b = 0.4; tree_marker.color.a = 0.6;
  tree_marker.pose.orientation.w = 1.0;

  // --- 算法参数 ---
  const double s_min = 0.15;    
  const double s_max = 0.50;    
  const int max_iter = 3000; 
  const double goal_tol_sq = 0.35 * 0.35;
  const double eta = 0.4, zeta = 0.15, d0 = 0.6;

  const double gx = goal.pose.position.x;
  const double gy = goal.pose.position.y;

  double init_dist_sq = std::pow(gx - start.pose.position.x, 2) + std::pow(gy - start.pose.position.y, 2);
  double current_min_dist_sq = init_dist_sq;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis_x(costmap_->getOriginX(), costmap_->getOriginX() + costmap_->getSizeInMetersX());
  std::uniform_real_distribution<> dis_y(costmap_->getOriginY(), costmap_->getOriginY() + costmap_->getSizeInMetersY());
  std::uniform_real_distribution<> dis_prob(0.0, 1.0);

  std::vector<Node> tree;
  tree.reserve(max_iter);
  tree.emplace_back(start.pose.position.x, start.pose.position.y, -1);

  bool found = false;
  for (int i = 0; i < max_iter; ++i) {
    // 1. 距离敏感目标偏置 (极低开销)
    double dist_ratio = std::sqrt(current_min_dist_sq / init_dist_sq);
    double p_bias = 0.5 - (0.4 * std::clamp(dist_ratio, 0.0, 1.0));

    double rx = (dis_prob(gen) < p_bias) ? gx : dis_x(gen);
    double ry = (dis_prob(gen) < p_bias) ? gy : dis_y(gen);

    // 2. 最近邻搜索 (单次遍历)
    int n_idx = 0;
    double min_d_sample_sq = 1e9;
    for (size_t j = 0; j < tree.size(); ++j) {
      double d_sq = std::pow(tree[j].x - rx, 2) + std::pow(tree[j].y - ry, 2);
      if (d_sq < min_d_sample_sq) { min_d_sample_sq = d_sq; n_idx = j; }
    }

    // 3. 轻量化自适应步长：基于 Costmap 代价直接映射 (核心优化点)
    unsigned int cur_mx, cur_my;
    double current_step = s_max;
    if (costmap_->worldToMap(tree[n_idx].x, tree[n_idx].y, cur_mx, cur_my)) {
      unsigned char cost = costmap_->getCost(cur_mx, cur_my);
      if (cost > 0) {
        // Cost 越高说明离障碍物越近，步长越小。253及以上为障碍物。
        // 使用 128 作为步长缩减的阈值，这是一个常见的膨胀层分界线
        double cost_factor = std::clamp(static_cast<double>(cost) / 128.0, 0.0, 1.0);
        current_step = s_max - (s_max - s_min) * cost_factor;
      }
    }

    double angle = std::atan2(ry - tree[n_idx].y, rx - tree[n_idx].x);
    double nx = tree[n_idx].x + current_step * std::cos(angle);
    double ny = tree[n_idx].y + current_step * std::sin(angle);

    // 4. 精简版 APF 斥力 (仅在有潜在危险时计算)
    double fx = zeta * (gx - nx);
    double fy = zeta * (gy - ny);
    unsigned int mx, my;
    if (costmap_->worldToMap(nx, ny, mx, my) && costmap_->getCost(mx, my) > 50) {
      int r = 2; // 缩小为 5x5 窗口
      for (int ox = -r; ox <= r; ox++) {
        for (int oy = -r; oy <= r; oy++) {
          unsigned int cx = mx + ox, cy = my + oy;
          if (cx < costmap_->getSizeInCellsX() && cy < costmap_->getSizeInCellsY()) {
            if (costmap_->getCost(cx, cy) >= 250) { 
              double wx, wy; costmap_->mapToWorld(cx, cy, wx, wy);
              double dx = nx - wx; double dy = ny - wy;
              double d_sq = dx*dx + dy*dy;
              if (d_sq < d0 * d0 && d_sq > 0.001) {
                double d = std::sqrt(d_sq);
                double rep = eta * (1.0/d - 1.0/d0) / (d_sq * d);
                fx += rep * dx; fy += rep * dy;
              }
            }
          }
        }
      }
    }
    nx += 0.1 * fx; ny += 0.1 * fy;

    // 5. 快速碰撞检测
    unsigned int fmx, fmy;
    if (!costmap_->worldToMap(nx, ny, fmx, fmy) || costmap_->getCost(fmx, fmy) >= 253) continue;

    // 6. 连线检测 (每隔 2 个 cell 检测一次以提速)
    bool collision = false;
    double dist_to_parent = std::hypot(nx - tree[n_idx].x, ny - tree[n_idx].y);
    int check_steps = static_cast<int>(dist_to_parent / (costmap_->getResolution() * 2.0));
    for (int k = 1; k <= check_steps; ++k) {
      double ratio = static_cast<double>(k) / check_steps;
      unsigned int cmx, cmy;
      if (costmap_->worldToMap(tree[n_idx].x + (nx - tree[n_idx].x) * ratio, 
                               tree[n_idx].y + (ny - tree[n_idx].y) * ratio, cmx, cmy)) {
        if (costmap_->getCost(cmx, cmy) >= 253) { collision = true; break; }
      }
    }
    if (collision) continue;

    // 7. 更新树与距离
    tree.emplace_back(nx, ny, n_idx);
    double d_to_goal_sq = std::pow(nx - gx, 2) + std::pow(ny - gy, 2);
    if (d_to_goal_sq < current_min_dist_sq) current_min_dist_sq = d_to_goal_sq;

    if (i % 2 == 0) { // 减少 Marker 绘制频率
      geometry_msgs::msg::Point p1, p2;
      p1.x = tree[n_idx].x; p1.y = tree[n_idx].y; p1.z = 0.1;
      p2.x = nx; p2.y = ny; p2.z = 0.1;
      tree_marker.points.push_back(p1); tree_marker.points.push_back(p2);
    }

    if (d_to_goal_sq < goal_tol_sq) { found = true; break; }
  }

  marker_pub_->publish(tree_marker);
  auto duration = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time_clock).count();

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
    RCLCPP_INFO(node_->get_logger(), "规划成功 | 采样节点数: %zu | 耗时: %.2f ms", tree.size(), duration);
    return global_path;
  }

  RCLCPP_WARN(node_->get_logger(), "规划失败 | 耗时: %.2f ms", duration);
  return global_path;
}

} // namespace nav2_rrt_adaptive_step_planner

PLUGINLIB_EXPORT_CLASS(nav2_rrt_adaptive_step_planner::RRTAdaptiveStepPlanner, nav2_core::GlobalPlanner)