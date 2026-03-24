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

  // 这里的 QoS 改为可靠传输，确保 RViz 稳定接收
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/rrt_tree_markers", 10);
  
  RCLCPP_INFO(node_->get_logger(), "RRTBSplineSmoothPlanner 已配置：窄通道优化版。");
}

void RRTBSplineSmoothPlanner::activate() { marker_pub_->on_activate(); }
void RRTBSplineSmoothPlanner::deactivate() { marker_pub_->on_deactivate(); }
void RRTBSplineSmoothPlanner::cleanup() { marker_pub_.reset(); }

bool RRTBSplineSmoothPlanner::isLineClear(double x1, double y1, double x2, double y2)
{
  double dist = std::hypot(x2 - x1, y2 - y1);
  double res = costmap_->getResolution();
  // 使用 0.5 倍分辨率步进，确保在窄道中不会漏掉任何一个像素点
  int steps = static_cast<int>(dist / (res * 0.5));
  
  for (int i = 0; i <= steps; ++i) {
    double t = (steps > 0) ? (static_cast<double>(i) / steps) : 1.0;
    double px = x1 + (x2 - x1) * t;
    double py = y1 + (y2 - y1) * t;
    
    unsigned int mx, my;
    if (!costmap_->worldToMap(px, py, mx, my)) return false;
    
    unsigned char cost = costmap_->getCost(mx, my);
    // 关键阈值：允许进入膨胀区（<253），拦截内切圆碰撞（253）和致命障碍（254）
    if (cost >= 253) return false;
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

      double b0 = (1.0 - 3.0*t + 3.0*t2 - t3) / 6.0;
      double b1 = (4.0 - 6.0*t2 + 3.0*t3) / 6.0;
      double b2 = (1.0 + 3.0*t + 3.0*t2 - 3.0*t3) / 6.0;
      double b3 = t3 / 6.0;

      geometry_msgs::msg::PoseStamped p;
      p.header = control_points[0].header;
      p.pose.position.x = b0 * control_points[i].pose.position.x + 
                          b1 * control_points[i+1].pose.position.x + 
                          b2 * control_points[i+2].pose.position.x + 
                          b3 * control_points[i+3].pose.position.x;
      p.pose.position.y = b0 * control_points[i].pose.position.y + 
                          b1 * control_points[i+1].pose.position.y + 
                          b2 * control_points[i+2].pose.position.y + 
                          b3 * control_points[i+3].pose.position.y;
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

  // 可视化 Marker 初始化
  visualization_msgs::msg::Marker tree_marker;
  tree_marker.header.frame_id = global_frame_;
  tree_marker.header.stamp = current_time;
  tree_marker.ns = "rrt_tree";
  tree_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  tree_marker.action = visualization_msgs::msg::Marker::ADD;
  tree_marker.scale.x = 0.015;
  tree_marker.color.r = 0.0; tree_marker.color.g = 1.0; tree_marker.color.b = 0.0; tree_marker.color.a = 0.6;

  std::vector<Node> tree;
  tree.emplace_back(start.pose.position.x, start.pose.position.y, -1);

  // 1. 局部采样范围优化：将随机点限制在起点和终点附近的矩形区域（加 3m 冗余）
  double min_x = std::min(start.pose.position.x, goal.pose.position.x) - 3.0;
  double max_x = std::max(start.pose.position.x, goal.pose.position.x) + 3.0;
  double min_y = std::min(start.pose.position.y, goal.pose.position.y) - 3.0;
  double max_y = std::max(start.pose.position.y, goal.pose.position.y) + 3.0;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis_x(min_x, max_x);
  std::uniform_real_distribution<> dis_y(min_y, max_y);
  std::uniform_real_distribution<> dis_prob(0.0, 1.0);

  bool found = false;
  const int max_iterations = 15000;
  const double step_size = 0.1; // 细化步长，保证能通过窄道
  const double goal_x = goal.pose.position.x;
  const double goal_y = goal.pose.position.y;

  for (int iterations = 0; iterations < max_iterations; ++iterations) {
    // 动态目标偏置
    double dist_to_goal_min = std::hypot(tree.back().x - goal_x, tree.back().y - goal_y);
    double goal_sample_prob = std::clamp(0.6 * std::exp(-0.4 * dist_to_goal_min), 0.1, 0.8);

    double rx, ry;
    if (dis_prob(gen) < goal_sample_prob) {
      rx = goal_x; ry = goal_y;
    } else {
      rx = dis_x(gen); ry = dis_y(gen);
    }

    int nearest_idx = 0; 
    double min_d = 1e9;
    for (size_t j = 0; j < tree.size(); ++j) {
      double d = std::hypot(tree[j].x - rx, tree[j].y - ry);
      if (d < min_d) { min_d = d; nearest_idx = j; }
    }

    double angle = std::atan2(ry - tree[nearest_idx].y, rx - tree[nearest_idx].x);
    double nx = tree[nearest_idx].x + step_size * std::cos(angle);
    double ny = tree[nearest_idx].y + step_size * std::sin(angle);

    if (isLineClear(tree[nearest_idx].x, tree[nearest_idx].y, nx, ny)) {
      tree.emplace_back(nx, ny, nearest_idx);
      
      geometry_msgs::msg::Point p1, p2;
      p1.x = tree[nearest_idx].x; p1.y = tree[nearest_idx].y; p1.z = 0.05;
      p2.x = nx; p2.y = ny; p2.z = 0.05;
      tree_marker.points.push_back(p1); tree_marker.points.push_back(p2);

      if (std::hypot(nx - goal_x, ny - goal_y) < 0.25) {
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

    // 2. 剪枝 (Pruning) - 产生最短折线路径
    std::vector<geometry_msgs::msg::PoseStamped> pruned;
    pruned.push_back(raw_path.front());
    for (size_t i = 0; i < raw_path.size(); ) {
      bool jumped = false;
      for (size_t j = raw_path.size() - 1; j > i; --j) {
        if (isLineClear(raw_path[i].pose.position.x, raw_path[i].pose.position.y,
                        raw_path[j].pose.position.x, raw_path[j].pose.position.y)) {
          pruned.push_back(raw_path[j]);
          i = j; jumped = true; break;
        }
      }
      if (!jumped) { i++; if(i < raw_path.size()) pruned.push_back(raw_path[i]); }
    }

    // 3. [防撞墙核心] B 样条平滑后进行二次碰撞检测
    if (pruned.size() < 4) {
      global_path.poses = pruned; // 点太少，平滑无意义，直接返回折线
    } else {
      auto smoothed_path = bsplineSmooth(pruned, 10);
      bool is_smooth_safe = true;
      for (size_t k = 0; k < smoothed_path.size() - 1; ++k) {
        if (!isLineClear(smoothed_path[k].pose.position.x, smoothed_path[k].pose.position.y,
                         smoothed_path[k+1].pose.position.x, smoothed_path[k+1].pose.position.y)) {
          is_smooth_safe = false; // 平滑路径切内角撞墙了
          break;
        }
      }
      // 如果平滑路径不安全，回退到剪枝后的折线路径（保证能过窄道）
      global_path.poses = is_smooth_safe ? smoothed_path : pruned;
    }

    auto end_time_clock = std::chrono::steady_clock::now();
    RCLCPP_INFO(node_->get_logger(), "RRT 成功 | 节点: %ld | 耗时: %.2f ms", 
                tree.size(), std::chrono::duration<double, std::milli>(end_time_clock - start_time_clock).count());
  }
  return global_path;
}

} // namespace nav2_rrt_bspline_smooth_planner

PLUGINLIB_EXPORT_CLASS(nav2_rrt_bspline_smooth_planner::RRTBSplineSmoothPlanner, nav2_core::GlobalPlanner)