#include "nav2_custom_planner/nav2_rrt_connect_auto_pruning_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <random>
#include <algorithm>
#include <chrono>
#include "pluginlib/class_list_macros.hpp"

namespace nav2_rrt_connect_auto_pruning_planner
{

void RRTConnectAutoPruningPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node in configure!");
  }

  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("/rrt_tree_markers", qos);
  
  RCLCPP_INFO(node->get_logger(), "RRTConnectAutoPruningPlanner (FSM自适应剪枝+详尽点位日志+B样条) 配置完成。");
}

void RRTConnectAutoPruningPlanner::activate() { if(marker_pub_) marker_pub_->on_activate(); }
void RRTConnectAutoPruningPlanner::deactivate() { if(marker_pub_) marker_pub_->on_deactivate(); }
void RRTConnectAutoPruningPlanner::cleanup() { marker_pub_.reset(); }

// 动态阈值碰撞检测
bool RRTConnectAutoPruningPlanner::isLineClear(double x1, double y1, double x2, double y2, unsigned char max_allowed_cost)
{
  double dist = std::hypot(x2 - x1, y2 - y1);
  double res = costmap_->getResolution();
  int steps = static_cast<int>(dist / res);
  
  for (int i = 0; i <= steps; ++i) {
    double t = static_cast<double>(i) / (steps > 0 ? steps : 1);
    double px = x1 + (x2 - x1) * t;
    double py = y1 + (y2 - y1) * t;
    unsigned int mx, my;
    if (!costmap_->worldToMap(px, py, mx, my) || costmap_->getCost(mx, my) >= max_allowed_cost) {
      return false;
    }
  }
  return true;
}

std::vector<geometry_msgs::msg::PoseStamped> RRTConnectAutoPruningPlanner::bsplineSmooth(
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

nav_msgs::msg::Path RRTConnectAutoPruningPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  auto node = node_.lock();
  if (!node) return nav_msgs::msg::Path();

  auto start_time_clock = std::chrono::steady_clock::now();
  auto current_time = node->now(); 

  nav_msgs::msg::Path global_path;
  global_path.header.stamp = current_time;
  global_path.header.frame_id = global_frame_;

  visualization_msgs::msg::Marker marker_start;
  marker_start.header.frame_id = global_frame_;
  marker_start.header.stamp = current_time;
  marker_start.ns = "rrt_tree_start";
  marker_start.id = 0;
  marker_start.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_start.action = visualization_msgs::msg::Marker::ADD;
  marker_start.scale.x = 0.015;
  marker_start.color.r = 0.0; marker_start.color.g = 0.0; marker_start.color.b = 1.0; marker_start.color.a = 0.6; // 蓝色

  visualization_msgs::msg::Marker marker_goal = marker_start;
  marker_goal.ns = "rrt_tree_goal";
  marker_goal.id = 1; 
  marker_goal.color.r = 1.0; marker_goal.color.g = 0.0; marker_goal.color.b = 0.0; marker_goal.color.a = 0.6; // 红色

  std::vector<Node> tree_start;
  std::vector<Node> tree_goal;
  
  const int max_iter = 10000; 
  tree_start.reserve(max_iter);
  tree_goal.reserve(max_iter);
  
  tree_start.emplace_back(start.pose.position.x, start.pose.position.y, -1);
  tree_goal.emplace_back(goal.pose.position.x, goal.pose.position.y, -1);

  std::vector<Node>* tree_A = &tree_start;
  std::vector<Node>* tree_B = &tree_goal;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis_x(costmap_->getOriginX(), costmap_->getOriginX() + costmap_->getSizeInMetersX());
  std::uniform_real_distribution<> dis_y(costmap_->getOriginY(), costmap_->getOriginY() + costmap_->getSizeInMetersY());
  std::uniform_real_distribution<> dis_prob(0.0, 1.0);

  bool found = false;
  const double step = 0.15; 
  const unsigned char TREE_GROWTH_COST = 253; // 允许在非致命区域内自由生长

  for (int iterations = 0; iterations < max_iter; ++iterations) {
    double iter_ratio = static_cast<double>(iterations) / max_iter;
    double p_bias = 0.1 + (0.4 * iter_ratio);

    double rx = (dis_prob(gen) < p_bias) ? (*tree_B)[0].x : dis_x(gen);
    double ry = (dis_prob(gen) < p_bias) ? (*tree_B)[0].y : dis_y(gen);

    int nearest_A = 0; double min_d_sq = 1e9;
    for (size_t j = 0; j < tree_A->size(); ++j) {
      double d_sq = std::pow((*tree_A)[j].x - rx, 2) + std::pow((*tree_A)[j].y - ry, 2);
      if (d_sq < min_d_sq) { min_d_sq = d_sq; nearest_A = j; }
    }

    double angle = std::atan2(ry - (*tree_A)[nearest_A].y, rx - (*tree_A)[nearest_A].x);
    double nx = (*tree_A)[nearest_A].x + step * std::cos(angle);
    double ny = (*tree_A)[nearest_A].y + step * std::sin(angle);

    if (isLineClear((*tree_A)[nearest_A].x, (*tree_A)[nearest_A].y, nx, ny, TREE_GROWTH_COST)) {
      tree_A->emplace_back(nx, ny, nearest_A);
      
      geometry_msgs::msg::Point p1, p2;
      p1.x = (*tree_A)[nearest_A].x; p1.y = (*tree_A)[nearest_A].y; p1.z = 0.1;
      p2.x = nx; p2.y = ny; p2.z = 0.1;
      
      auto& active_marker = (tree_A == &tree_start) ? marker_start : marker_goal;
      active_marker.points.push_back(p1); active_marker.points.push_back(p2);

      int nearest_B = 0; min_d_sq = 1e9;
      for (size_t j = 0; j < tree_B->size(); ++j) {
        double d_sq = std::pow((*tree_B)[j].x - nx, 2) + std::pow((*tree_B)[j].y - ny, 2);
        if (d_sq < min_d_sq) { min_d_sq = d_sq; nearest_B = j; }
      }

      if (isLineClear((*tree_B)[nearest_B].x, (*tree_B)[nearest_B].y, nx, ny, TREE_GROWTH_COST)) {
        tree_B->emplace_back(nx, ny, nearest_B);
        
        geometry_msgs::msg::Point p3, p4;
        p3.x = (*tree_B)[nearest_B].x; p3.y = (*tree_B)[nearest_B].y; p3.z = 0.1;
        p4.x = nx; p4.y = ny; p4.z = 0.1;
        
        auto& passive_marker = (tree_B == &tree_start) ? marker_start : marker_goal;
        passive_marker.points.push_back(p3); passive_marker.points.push_back(p4);
        
        found = true; 
        break;
      } else {
        double angle_B = std::atan2(ny - (*tree_B)[nearest_B].y, nx - (*tree_B)[nearest_B].x);
        double nbx = (*tree_B)[nearest_B].x + step * std::cos(angle_B);
        double nby = (*tree_B)[nearest_B].y + step * std::sin(angle_B);
        if (isLineClear((*tree_B)[nearest_B].x, (*tree_B)[nearest_B].y, nbx, nby, TREE_GROWTH_COST)) {
          tree_B->emplace_back(nbx, nby, nearest_B);
          
          geometry_msgs::msg::Point p3, p4;
          p3.x = (*tree_B)[nearest_B].x; p3.y = (*tree_B)[nearest_B].y; p3.z = 0.1;
          p4.x = nbx; p4.y = nby; p4.z = 0.1;
          
          auto& passive_marker = (tree_B == &tree_start) ? marker_start : marker_goal;
          passive_marker.points.push_back(p3); passive_marker.points.push_back(p4);
        }
      }
    }
    std::swap(tree_A, tree_B);
  }

  if(marker_pub_) {
    marker_pub_->publish(marker_start);
    marker_pub_->publish(marker_goal);
  }

  if (found) {
    std::vector<geometry_msgs::msg::PoseStamped> raw_path;
    
    std::vector<geometry_msgs::msg::PoseStamped> path_start;
    int curr_s = tree_start.size() - 1;
    while (curr_s != -1) {
      geometry_msgs::msg::PoseStamped p;
      p.header = global_path.header;
      p.pose.position.x = tree_start[curr_s].x; p.pose.position.y = tree_start[curr_s].y;
      p.pose.orientation.w = 1.0;
      path_start.push_back(p);
      curr_s = tree_start[curr_s].parent_idx;
    }
    std::reverse(path_start.begin(), path_start.end());

    std::vector<geometry_msgs::msg::PoseStamped> path_goal;
    int curr_g = tree_goal.back().parent_idx; 
    while (curr_g != -1) {
      geometry_msgs::msg::PoseStamped p;
      p.header = global_path.header;
      p.pose.position.x = tree_goal[curr_g].x; p.pose.position.y = tree_goal[curr_g].y;
      p.pose.orientation.w = 1.0;
      path_goal.push_back(p);
      curr_g = tree_goal[curr_g].parent_idx;
    }

    raw_path.insert(raw_path.end(), path_start.begin(), path_start.end());
    raw_path.insert(raw_path.end(), path_goal.begin(), path_goal.end());

    // =========================================================================
    // ✨ FSM (有限状态机) 自适应剪枝：带详尽点位追踪日志
    // =========================================================================
    std::vector<geometry_msgs::msg::PoseStamped> adaptive_pruned;
    if (!raw_path.empty()) {
      adaptive_pruned.push_back(raw_path.front());
      size_t curr_idx = 0;
      
      // 【调参核心】安全阈值
      const unsigned char SAFE_THRESHOLD = 20; 

      int state_a_count = 0;
      int state_b_count = 0;
      unsigned char max_cost_in_path = 0;

      RCLCPP_INFO(node->get_logger(), "--- [FSM 剪枝开始] 原始路径共 %zu 个点，当前阈值: %d ---", raw_path.size(), SAFE_THRESHOLD);

      while (curr_idx < raw_path.size() - 1) {
        unsigned int mx, my;
        unsigned char curr_cost = 0;
        if (costmap_->worldToMap(raw_path[curr_idx].pose.position.x, raw_path[curr_idx].pose.position.y, mx, my)) {
          curr_cost = costmap_->getCost(mx, my);
          if (curr_cost > max_cost_in_path) max_cost_in_path = curr_cost;
        }

        bool currently_safe = (curr_cost < SAFE_THRESHOLD);

        // --- 每次点进去都打印状态 ---
        if (currently_safe) {
          RCLCPP_INFO(node->get_logger(), "  -> 处理点[%zu] | Cost: %d | [状态A: 安全区] 允许剪枝", curr_idx, curr_cost);
        } else {
          RCLCPP_INFO(node->get_logger(), "  -> 处理点[%zu] | Cost: %d | [状态B: 危险区] 强制关闭剪枝", curr_idx, curr_cost);
        }

        // 状态机核心判断
        if (currently_safe) {
          // 【状态 A】执行贪婪剪枝
          state_a_count++;
          bool jump = false;
          for (size_t t = raw_path.size() - 1; t > curr_idx; --t) {
            if (isLineClear(raw_path[curr_idx].pose.position.x, raw_path[curr_idx].pose.position.y,
                            raw_path[t].pose.position.x, raw_path[t].pose.position.y, SAFE_THRESHOLD)) {
              
              // 若成功产生跨点跳跃（即发生了剪枝），也打个日志看个明白
              if (t > curr_idx + 1) {
                RCLCPP_INFO(node->get_logger(), "     ==> 触发剪枝！直接从点[%zu] 跳跃连接至点[%zu]", curr_idx, t);
              }
              
              adaptive_pruned.push_back(raw_path[t]);
              curr_idx = t; 
              jump = true; 
              break;
            }
          }
          if (!jump) { 
            curr_idx++; 
            adaptive_pruned.push_back(raw_path[curr_idx]); 
          }
        } else {
          // 【状态 B】保留原始点
          state_b_count++;
          curr_idx++;
          adaptive_pruned.push_back(raw_path[curr_idx]);
        }
      }

      RCLCPP_INFO(node->get_logger(), "--- [FSM 总结] Max Cost: %d | 状态A(剪枝)总计: %d 次 | 状态B(保留)总计: %d 次 | 输出路径点数: %zu ---", 
                  max_cost_in_path, state_a_count, state_b_count, adaptive_pruned.size());
    }

    // B样条平滑
    std::vector<geometry_msgs::msg::PoseStamped> control_points = adaptive_pruned;
    if (control_points.size() >= 2) {
      control_points.insert(control_points.begin(), 2, adaptive_pruned.front());
      control_points.insert(control_points.end(), 2, adaptive_pruned.back());
    }

    global_path.poses = bsplineSmooth(control_points, 10);
    if (global_path.poses.empty()) global_path.poses = adaptive_pruned;

    auto duration = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time_clock).count();
    RCLCPP_INFO(node->get_logger(), "FSM 自适应规划成功 | 耗时: %.2f ms", duration);
  } else {
    auto duration = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time_clock).count();
    RCLCPP_WARN(node->get_logger(), "规划超时失败 | 耗时: %.2f ms", duration);
  }

  return global_path;
}

} // namespace nav2_rrt_connect_auto_pruning_planner

PLUGINLIB_EXPORT_CLASS(nav2_rrt_connect_auto_pruning_planner::RRTConnectAutoPruningPlanner, nav2_core::GlobalPlanner)