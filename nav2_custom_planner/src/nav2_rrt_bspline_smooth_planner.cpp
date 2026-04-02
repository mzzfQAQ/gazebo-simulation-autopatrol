#include "nav2_custom_planner/nav2_rrt_bspline_smooth_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <random>
#include <algorithm>
#include <chrono>
#include "pluginlib/class_list_macros.hpp"

namespace nav2_rrt_bspline_smooth_planner
{

void RRTBSplineSmoothPlanner::configure(
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
  
  RCLCPP_INFO(node_->get_logger(), "RRTBSplineSmoothPlanner (迭代敏感偏置 + 剪枝 + B样条) 已配置。");
}

void RRTBSplineSmoothPlanner::activate() { marker_pub_->on_activate(); }
void RRTBSplineSmoothPlanner::deactivate() { marker_pub_->on_deactivate(); }
void RRTBSplineSmoothPlanner::cleanup() { marker_pub_.reset(); }

bool RRTBSplineSmoothPlanner::isLineClear(double x1, double y1, double x2, double y2)
{
  double dist = std::hypot(x2 - x1, y2 - y1);
  double res = costmap_->getResolution();
  int steps = static_cast<int>(dist / res);
  for (int i = 0; i <= steps; ++i) {
    double px = x1 + (x2 - x1) * (static_cast<double>(i) / (steps > 0 ? steps : 1));
    double py = y1 + (y2 - y1) * (static_cast<double>(i) / (steps > 0 ? steps : 1));
    unsigned int mx, my;
    if (!costmap_->worldToMap(px, py, mx, my) || costmap_->getCost(mx, my) >= 253) {
      return false;
    }
  }
  return true;
}

std::vector<geometry_msgs::msg::PoseStamped> RRTBSplineSmoothPlanner::bsplineSmooth(
  const std::vector<geometry_msgs::msg::PoseStamped> & control_points,
  int sample_num)
{
  std::vector<geometry_msgs::msg::PoseStamped> smooth_path;
  int n = control_points.size();

  if (n < 4) return control_points;

  for (int i = 0; i < n - 3; ++i) {
    for (int j = 0; j <= sample_num; ++j) {
      double t = static_cast<double>(j) / sample_num;
      double t2 = t * t;
      double t3 = t * t * t;

      double b0 = (1.0 - 3.0 * t + 3.0 * t2 - t3) / 6.0;
      double b1 = (4.0 - 6.0 * t2 + 3.0 * t3) / 6.0;
      double b2 = (1.0 + 3.0 * t + 3.0 * t2 - 3.0 * t3) / 6.0;
      double b3 = t3 / 6.0;

      geometry_msgs::msg::PoseStamped p;
      p.header = control_points[0].header;
      p.pose.position.x = b0 * control_points[i].pose.position.x +
                          b1 * control_points[i + 1].pose.position.x +
                          b2 * control_points[i + 2].pose.position.x +
                          b3 * control_points[i + 3].pose.position.x;
      p.pose.position.y = b0 * control_points[i].pose.position.y +
                          b1 * control_points[i + 1].pose.position.y +
                          b2 * control_points[i + 2].pose.position.y +
                          b3 * control_points[i + 3].pose.position.y;
      p.pose.orientation.w = 1.0;
      smooth_path.push_back(p);
    }
  }
  return smooth_path;
}

nav_msgs::msg::Path RRTBSplineSmoothPlanner::createPlan(
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
  tree_marker.color.r = 0.0; tree_marker.color.g = 1.0; tree_marker.color.b = 0.0; tree_marker.color.a = 0.4;

  const double gx = goal.pose.position.x;
  const double gy = goal.pose.position.y;

  std::vector<Node> tree;
  tree.reserve(2000);
  tree.emplace_back(start.pose.position.x, start.pose.position.y, -1);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis_x(costmap_->getOriginX(), costmap_->getOriginX() + costmap_->getSizeInMetersX());
  std::uniform_real_distribution<> dis_y(costmap_->getOriginY(), costmap_->getOriginY() + costmap_->getSizeInMetersY());
  std::uniform_real_distribution<> dis_prob(0.0, 1.0);

  bool found = false;
  const int max_iter = 2000;
  int iterations = 0;

  for (; iterations < max_iter; ++iterations) {
    // --- 核心修改：迭代次数敏感型动态目标偏置 ---
    // 逻辑：随着迭代次数增加，p_bias 从 0.1 线性增加到 0.5
    double iter_ratio = static_cast<double>(iterations) / max_iter;
    double p_bias = 0.1 + (0.4 * iter_ratio);

    double rx = (dis_prob(gen) < p_bias) ? gx : dis_x(gen);
    double ry = (dis_prob(gen) < p_bias) ? gy : dis_y(gen);

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
      
      geometry_msgs::msg::Point p1, p2;
      p1.x = tree[nearest].x; p1.y = tree[nearest].y; p1.z = 0.1;
      p2.x = nx; p2.y = ny; p2.z = 0.1;
      tree_marker.points.push_back(p1); tree_marker.points.push_back(p2);

      double d_to_goal_sq = std::pow(nx - gx, 2) + std::pow(ny - gy, 2);
      if (d_to_goal_sq < 0.16) { 
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

    geometry_msgs::msg::PoseStamped exact_goal = goal;
    exact_goal.header = global_path.header;
    if (std::hypot(raw_path.back().pose.position.x - gx, raw_path.back().pose.position.y - gy) > 0.05) {
        raw_path.push_back(exact_goal);
    }

    // 1. 剪枝逻辑 (Greedy Pruning)
    std::vector<geometry_msgs::msg::PoseStamped> pruned;
    if (!raw_path.empty()) {
      pruned.push_back(raw_path.front());
      size_t curr_idx = 0;
      while (curr_idx < raw_path.size() - 1) {
        bool jump = false;
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

    // 2. B 样条平滑
    std::vector<geometry_msgs::msg::PoseStamped> control_points = pruned;
    
    if (control_points.size() >= 2) {
      control_points.insert(control_points.begin(), 2, pruned.front());
      control_points.insert(control_points.end(), 2, pruned.back());
    }

    global_path.poses = bsplineSmooth(control_points, 10);

    if (global_path.poses.empty()) {
      global_path.poses = pruned;
    }

    auto duration = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time_clock).count();
    RCLCPP_INFO(node_->get_logger(), 
      "RRT-B样条平滑 成功 | 原始节点: %ld | 剪枝后: %ld | 平滑节点: %ld | 耗时: %.2f ms", 
      raw_path.size(), pruned.size(), global_path.poses.size(), duration);
  } else {
    auto duration = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time_clock).count();
    RCLCPP_WARN(node_->get_logger(), "RRT-B样条平滑 失败 | 耗时: %.2f ms", duration);
  }

  return global_path;
}

} // namespace nav2_rrt_bspline_smooth_planner

PLUGINLIB_EXPORT_CLASS(nav2_rrt_bspline_smooth_planner::RRTBSplineSmoothPlanner, nav2_core::GlobalPlanner)