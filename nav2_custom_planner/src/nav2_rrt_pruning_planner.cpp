#include "nav2_custom_planner/nav2_rrt_pruning_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <random>
#include <algorithm>
#include <chrono>
#include "pluginlib/class_list_macros.hpp"

namespace nav2_rrt_pruning_planner
{

void RRTPruningPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/rrt_tree_markers", qos);
  
  RCLCPP_INFO(node_->get_logger(), "RRTPruningPlanner (距离敏感偏置版) 已配置。");
}

void RRTPruningPlanner::activate() { marker_pub_->on_activate(); }
void RRTPruningPlanner::deactivate() { marker_pub_->on_deactivate(); }
void RRTPruningPlanner::cleanup() { marker_pub_.reset(); }

bool RRTPruningPlanner::isLineClear(double x1, double y1, double x2, double y2)
{
  double dist = std::hypot(x2 - x1, y2 - y1);
  double res = costmap_->getResolution();
  int steps = static_cast<int>(dist / res);
  for (int i = 0; i <= steps; ++i) {
    double px = x1 + (x2 - x1) * (static_cast<double>(i) / (steps > 0 ? steps : 1));
    double py = y1 + (y2 - y1) * (static_cast<double>(i) / (steps > 0 ? steps : 1));
    unsigned int mx, my;
    if (!costmap_->worldToMap(px, py, mx, my) || costmap_->getCost(mx, my) >= 180) {
      return false;
    }
  }
  return true;
}

nav_msgs::msg::Path RRTPruningPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  auto start_time_clock = std::chrono::steady_clock::now();
  auto current_time = node_->now(); 

  nav_msgs::msg::Path global_path;
  global_path.header.stamp = current_time;
  global_path.header.frame_id = global_frame_;

  visualization_msgs::msg::Marker tree_marker;
  tree_marker.header.frame_id = global_frame_;
  tree_marker.header.stamp = current_time;
  tree_marker.ns = "rrt_tree";
  tree_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  tree_marker.action = visualization_msgs::msg::Marker::ADD;
  tree_marker.scale.x = 0.015;
  tree_marker.color.r = 0.0; tree_marker.color.g = 0.8; tree_marker.color.b = 1.0; tree_marker.color.a = 0.6;

  // 目标点缓存
  const double gx = goal.pose.position.x;
  const double gy = goal.pose.position.y;

  std::vector<Node> tree;
  tree.reserve(2000);
  tree.emplace_back(start.pose.position.x, start.pose.position.y, -1);

  // 初始化距离用于偏置计算
  double init_dist_sq = std::pow(gx - tree[0].x, 2) + std::pow(gy - tree[0].y, 2);
  double current_min_dist_sq = init_dist_sq;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis_x(costmap_->getOriginX(), costmap_->getOriginX() + costmap_->getSizeInMetersX());
  std::uniform_real_distribution<> dis_y(costmap_->getOriginY(), costmap_->getOriginY() + costmap_->getSizeInMetersY());
  std::uniform_real_distribution<> dis_prob(0.0, 1.0);

  bool found = false;
  const int max_iter = 2000;
  int iterations = 0;

  for (; iterations < max_iter; ++iterations) {
    // --- 核心改进：距离敏感型动态目标偏置 ---
    // 逻辑：离目标越近，采样目标的概率越大 (从 0.1 线性增加到 0.5)
    double dist_ratio = std::sqrt(current_min_dist_sq / init_dist_sq);
    double p_bias = 0.5 - (0.4 * std::clamp(dist_ratio, 0.0, 1.0));

    double rx = (dis_prob(gen) < p_bias) ? gx : dis_x(gen);
    double ry = (dis_prob(gen) < p_bias) ? gy : dis_y(gen);

    // 最近邻搜索 (平方距离避开开方)
    int nearest = 0; double min_d_sq = 1e9;
    for (size_t j = 0; j < tree.size(); ++j) {
      double d_sq = std::pow(tree[j].x - rx, 2) + std::pow(tree[j].y - ry, 2);
      if (d_sq < min_d_sq) { min_d_sq = d_sq; nearest = j; }
    }

    double step = 0.3;
    double angle = std::atan2(ry - tree[nearest].y, rx - tree[nearest].x);
    double nx = tree[nearest].x + step * std::cos(angle);
    double ny = tree[nearest].y + step * std::sin(angle);

    if (isLineClear(tree[nearest].x, tree[nearest].y, nx, ny)) {
      tree.emplace_back(nx, ny, nearest);
      
      // 更新当前树离目标的最近记录
      double d_to_goal_sq = std::pow(nx - gx, 2) + std::pow(ny - gy, 2);
      if (d_to_goal_sq < current_min_dist_sq) {
        current_min_dist_sq = d_to_goal_sq;
      }

      geometry_msgs::msg::Point p1, p2;
      p1.x = tree[nearest].x; p1.y = tree[nearest].y; p1.z = 0.1;
      p2.x = nx; p2.y = ny; p2.z = 0.1;
      tree_marker.points.push_back(p1); tree_marker.points.push_back(p2);

      if (d_to_goal_sq < 0.16) { // 0.4m 的平方
        found = true; break;
      }
    }
  }

  marker_pub_->publish(tree_marker);

  if (found) {
    std::vector<geometry_msgs::msg::PoseStamped> raw_path;
    int curr = static_cast<int>(tree.size()) - 1;
    while (curr != -1) {
      geometry_msgs::msg::PoseStamped p;
      p.header = global_path.header;
      p.pose.position.x = tree[curr].x; p.pose.position.y = tree[curr].y;
      p.pose.orientation.w = 1.0;
      raw_path.push_back(p);
      curr = tree[curr].parent_idx;
    }
    std::reverse(raw_path.begin(), raw_path.end());

    // 1. 剪枝逻辑 (Greedy Pruning)
    std::vector<geometry_msgs::msg::PoseStamped> pruned;
    if (!raw_path.empty()) {
      pruned.push_back(raw_path.front());
      size_t curr_idx = 0;
      while (curr_idx < raw_path.size() - 1) {
        bool jump = false;
        // 从最后开始向前找第一个能直连的点
        for (size_t t = raw_path.size() - 1; t > curr_idx; --t) {
          if (isLineClear(raw_path[curr_idx].pose.position.x, raw_path[curr_idx].pose.position.y,
                          raw_path[t].pose.position.x, raw_path[t].pose.position.y)) {
            pruned.push_back(raw_path[t]);
            curr_idx = t; jump = true; break;
          }
        }
        if (!jump) { curr_idx++; pruned.push_back(raw_path[curr_idx]); }
      }
    }

    // 2. 线性插值优化 (确保路径点密度均匀)
    std::vector<geometry_msgs::msg::PoseStamped> interpolated_path;
    const double res = 0.1; 
    if (!pruned.empty()) {
      for (size_t i = 0; i < pruned.size() - 1; ++i) {
        interpolated_path.push_back(pruned[i]);
        double d = std::hypot(pruned[i+1].pose.position.x - pruned[i].pose.position.x,
                              pruned[i+1].pose.position.y - pruned[i].pose.position.y);
        if (d > res) {
          int n = static_cast<int>(d / res);
          for (int j = 1; j < n; ++j) {
            geometry_msgs::msg::PoseStamped p = pruned[i];
            double r = static_cast<double>(j) / n;
            p.pose.position.x = pruned[i].pose.position.x + r * (pruned[i+1].pose.position.x - pruned[i].pose.position.x);
            p.pose.position.y = pruned[i].pose.position.y + r * (pruned[i+1].pose.position.y - pruned[i].pose.position.y);
            interpolated_path.push_back(p);
          }
        }
      }
      interpolated_path.push_back(pruned.back());
    }
    global_path.poses = interpolated_path;

    auto duration = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time_clock).count();
    RCLCPP_INFO(node_->get_logger(), 
      "Pruning-RRT 成功 | 节点: %ld | 耗时: %.2f ms | 剪枝后采样节点数: %ld", 
      raw_path.size(), duration, global_path.poses.size());
  } else {
    auto duration = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time_clock).count();
    RCLCPP_WARN(node_->get_logger(), "Pruning-RRT 失败 | 耗时: %.2f ms", duration);
  }

  return global_path;
}

} // namespace nav2_rrt_pruning_planner

PLUGINLIB_EXPORT_CLASS(nav2_rrt_pruning_planner::RRTPruningPlanner, nav2_core::GlobalPlanner)