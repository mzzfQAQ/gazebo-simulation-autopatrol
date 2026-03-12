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
  
  RCLCPP_INFO(node_->get_logger(), "RRTBSplineSmoothPlanner 已配置：集成距离敏感型目标偏置。");
}

void RRTBSplineSmoothPlanner::activate() { marker_pub_->on_activate(); }
void RRTBSplineSmoothPlanner::deactivate() { marker_pub_->on_deactivate(); }
void RRTBSplineSmoothPlanner::cleanup() { marker_pub_.reset(); }

bool RRTBSplineSmoothPlanner::isLineClear(double x1, double y1, double x2, double y2)
{
  double dist = std::hypot(x2 - x1, y2 - y1);
  int steps = static_cast<int>(dist / costmap_->getResolution());
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

  visualization_msgs::msg::Marker tree_marker;
  tree_marker.header.frame_id = global_frame_;
  tree_marker.header.stamp = current_time;
  tree_marker.ns = "rrt_tree";
  tree_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  tree_marker.action = visualization_msgs::msg::Marker::ADD;
  tree_marker.scale.x = 0.02;
  tree_marker.color.r = 0.0; tree_marker.color.g = 1.0; tree_marker.color.b = 0.0; tree_marker.color.a = 0.5;

  std::vector<Node> tree;
  tree.emplace_back(start.pose.position.x, start.pose.position.y, -1);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis_x(costmap_->getOriginX(), costmap_->getOriginX() + costmap_->getSizeInMetersX());
  std::uniform_real_distribution<> dis_y(costmap_->getOriginY(), costmap_->getOriginY() + costmap_->getSizeInMetersY());
  std::uniform_real_distribution<> dis_prob(0.0, 1.0);

  bool found = false;
  const int max_iterations = 3000;
  const double goal_x = goal.pose.position.x;
  const double goal_y = goal.pose.position.y;

  for (int iterations = 0; iterations < max_iterations; ++iterations) {
    // 1. 寻找当前树中距离目标最近的点，以计算动态偏置概率
    double dist_to_goal_min = 1e9;
    int nearest_to_target_idx = 0;
    for (size_t j = 0; j < tree.size(); ++j) {
        double d = std::hypot(tree[j].x - goal_x, tree[j].y - goal_y);
        if (d < dist_to_goal_min) {
            dist_to_goal_min = d;
            nearest_to_target_idx = j;
        }
    }

    // 2. 计算动态目标偏置概率 (Distance-sensitive Goal Bias)
    // 距离越短，概率越高。公式：P = max_p * exp(-k * dist)
    // 这里使用简单的线性映射：如果在 5米外，概率为 0.05；如果在 0.5米内，概率为 0.5
    double goal_sample_prob = 0.5 * std::exp(-0.4 * dist_to_goal_min); 
    goal_sample_prob = std::clamp(goal_sample_prob, 0.05, 0.7); // 限制在 5% 到 70% 之间

    double rx, ry;
    if (dis_prob(gen) < goal_sample_prob) {
      rx = goal_x;
      ry = goal_y;
    } else {
      rx = dis_x(gen);
      ry = dis_y(gen);
    }

    // 3. 寻找 tree 中距离采样点最近的节点
    int nearest_node_idx = 0; 
    double min_d = 1e9;
    for (size_t j = 0; j < tree.size(); ++j) {
      double d = std::hypot(tree[j].x - rx, tree[j].y - ry);
      if (d < min_d) { min_d = d; nearest_node_idx = j; }
    }

    // 4. 步进增长
    double step = 0.3;
    double angle = std::atan2(ry - tree[nearest_node_idx].y, rx - tree[nearest_node_idx].x);
    double nx = tree[nearest_node_idx].x + step * std::cos(angle);
    double ny = tree[nearest_node_idx].y + step * std::sin(angle);

    if (isLineClear(tree[nearest_node_idx].x, tree[nearest_node_idx].y, nx, ny)) {
      tree.emplace_back(nx, ny, nearest_node_idx);
      geometry_msgs::msg::Point p1, p2;
      p1.x = tree[nearest_node_idx].x; p1.y = tree[nearest_node_idx].y; p1.z = 0.1;
      p2.x = nx; p2.y = ny; p2.z = 0.1;
      tree_marker.points.push_back(p1); tree_marker.points.push_back(p2);

      if (std::hypot(nx - goal_x, ny - goal_y) < 0.4) {
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

    // 剪枝逻辑
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

    // B 样条平滑
    std::vector<geometry_msgs::msg::PoseStamped> control_points = pruned;
    if (control_points.size() >= 2) {
        control_points.insert(control_points.begin(), pruned.front());
        control_points.push_back(pruned.back());
    }
    
    global_path.poses = bsplineSmooth(control_points, 10);

    auto end_time_clock = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> elapsed_ms = end_time_clock - start_time_clock;

    RCLCPP_INFO(node_->get_logger(), 
      "RRT(距离敏感偏置) 成功 | 剪枝控制点: %ld | 采样点: %ld | 耗时: %.2f ms", 
      pruned.size(), global_path.poses.size(), elapsed_ms.count());
  }

  return global_path;
}

} // namespace nav2_rrt_bspline_smooth_planner

PLUGINLIB_EXPORT_CLASS(nav2_rrt_bspline_smooth_planner::RRTBSplineSmoothPlanner, nav2_core::GlobalPlanner)