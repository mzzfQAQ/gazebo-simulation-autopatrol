#include "nav2_custom_planner/nav2_rrt_connect_smooth_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_rrt_connect_smooth_planner
{

void RRTConnectSmoothPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  logger_ = node->get_logger();
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  name_ = name;

  nav2_util::declare_parameter_if_not_declared(node, name + ".step_size", rclcpp::ParameterValue(0.2));
  node->get_parameter(name + ".step_size", step_size_);
  nav2_util::declare_parameter_if_not_declared(node, name + ".max_iterations", rclcpp::ParameterValue(5000));
  node->get_parameter(name + ".max_iterations", max_iterations_);
  nav2_util::declare_parameter_if_not_declared(node, name + ".connection_threshold", rclcpp::ParameterValue(0.5));
  node->get_parameter(name + ".connection_threshold", connection_threshold_);
  nav2_util::declare_parameter_if_not_declared(node, name + ".interpolation_resolution", rclcpp::ParameterValue(0.05));
  node->get_parameter(name + ".interpolation_resolution", interpolation_resolution_);
  
  marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
  RCLCPP_INFO(logger_, "RRTConnectSmoothPlanner: Configured.");
}

void RRTConnectSmoothPlanner::cleanup() { marker_pub_.reset(); }
void RRTConnectSmoothPlanner::activate() { }
void RRTConnectSmoothPlanner::deactivate() { }

nav_msgs::msg::Path RRTConnectSmoothPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  costmap_ = costmap_ros_->getCostmap();
  nav_msgs::msg::Path global_path;
  global_path.header.stamp = rclcpp::Clock().now();
  global_path.header.frame_id = start.header.frame_id;

  // 初始化双树
  std::vector<Node> start_tree, goal_tree;
  start_tree.push_back({start.pose.position.x, start.pose.position.y, -1});
  goal_tree.push_back({goal.pose.position.x, goal.pose.position.y, -1});

  std::default_random_engine gen((std::random_device())());
  std::uniform_real_distribution<double> dis_x(costmap_->getOriginX(), costmap_->getOriginX() + costmap_->getSizeInMetersX());
  std::uniform_real_distribution<double> dis_y(costmap_->getOriginY(), costmap_->getOriginY() + costmap_->getSizeInMetersY());

  bool connected = false;
  int start_node_final_idx = -1;
  int goal_node_final_idx = -1;
  bool start_is_A = true; 

  // --- 核心搜索循环：完全保留你的原始逻辑 ---
  for (int i = 0; i < max_iterations_; ++i) {
    Node rand_node = {dis_x(gen), dis_y(gen), -1};
    Node new_node_current;

    if (extendTree(start_tree, rand_node, new_node_current)) {
      Node new_node_other;
      // 尝试让树 B 向树 A 的最新节点连接
      if (extendTree(goal_tree, start_tree.back(), new_node_other)) {
        if (getDistance(start_tree.back(), goal_tree.back()) < connection_threshold_) {
          connected = true;
          start_node_final_idx = start_is_A ? (static_cast<int>(start_tree.size()) - 1) : (static_cast<int>(goal_tree.size()) - 1);
          goal_node_final_idx = start_is_A ? (static_cast<int>(goal_tree.size()) - 1) : (static_cast<int>(start_tree.size()) - 1);
          break;
        }
      }
    }
    std::swap(start_tree, goal_tree);
    start_is_A = !start_is_A;
  }

  publishTree(start_is_A ? start_tree : goal_tree, start_is_A ? goal_tree : start_tree, start.header.frame_id);

  if (connected) {
    std::vector<Node>& real_start_tree = start_is_A ? start_tree : goal_tree;
    std::vector<Node>& real_goal_tree = start_is_A ? goal_tree : start_tree;

    // 回溯路径
    int curr = start_node_final_idx;
    while (curr != -1) {
      geometry_msgs::msg::PoseStamped p;
      p.pose.position.x = real_start_tree[curr].x; p.pose.position.y = real_start_tree[curr].y;
      global_path.poses.push_back(p);
      curr = real_start_tree[curr].parent_idx;
    }
    std::reverse(global_path.poses.begin(), global_path.poses.end());

    curr = goal_node_final_idx;
    while (curr != -1) {
      geometry_msgs::msg::PoseStamped p;
      p.pose.position.x = real_goal_tree[curr].x; p.pose.position.y = real_goal_tree[curr].y;
      global_path.poses.push_back(p);
      curr = real_goal_tree[curr].parent_idx;
    }

    // --- 只有在成功生成原始路径后才进行优化 ---
    if (global_path.poses.size() > 2) {
      optimizePath(global_path); // 剪枝
      smoothPath(global_path, interpolation_resolution_); // 平滑
    }

    // --- 计算朝向：这是 Nav2 控制器成功的关键 ---
    for (size_t i = 0; i < global_path.poses.size(); ++i) {
      double yaw = (i < global_path.poses.size() - 1) ? 
        atan2(global_path.poses[i+1].pose.position.y - global_path.poses[i].pose.position.y,
              global_path.poses[i+1].pose.position.x - global_path.poses[i].pose.position.x) : 
        tf2::getYaw(goal.pose.orientation);
      
      tf2::Quaternion q; q.setRPY(0, 0, yaw);
      global_path.poses[i].pose.orientation = tf2::toMsg(q);
      global_path.poses[i].header = global_path.header;
    }
    RCLCPP_INFO(logger_, "Path created with %zu points", global_path.poses.size());
  }

  return global_path;
}

bool RRTConnectSmoothPlanner::extendTree(std::vector<Node> & tree, const Node & target, Node & new_node) {
  int nearest_idx = 0;
  double min_dist = 1e9;
  for (size_t i = 0; i < tree.size(); ++i) {
    double d = getDistance(tree[i], target);
    if (d < min_dist) { min_dist = d; nearest_idx = static_cast<int>(i); }
  }
  double theta = atan2(target.y - tree[nearest_idx].y, target.x - tree[nearest_idx].x);
  new_node.x = tree[nearest_idx].x + step_size_ * cos(theta);
  new_node.y = tree[nearest_idx].y + step_size_ * sin(theta);
  new_node.parent_idx = nearest_idx;
  if (isCollisionFree(tree[nearest_idx], new_node)) {
    tree.push_back(new_node); return true;
  }
  return false;
}

void RRTConnectSmoothPlanner::optimizePath(nav_msgs::msg::Path & path) {
  if (path.poses.size() < 3) return;
  std::vector<geometry_msgs::msg::PoseStamped> opt;
  opt.push_back(path.poses.front());
  size_t curr = 0;
  while (curr < path.poses.size() - 1) {
    size_t best = curr + 1;
    for (size_t i = path.poses.size() - 1; i > curr + 1; --i) {
      Node n1 = {path.poses[curr].pose.position.x, path.poses[curr].pose.position.y, -1};
      Node n2 = {path.poses[i].pose.position.x, path.poses[i].pose.position.y, -1};
      if (isCollisionFree(n1, n2)) { best = i; break; }
    }
    opt.push_back(path.poses[best]);
    curr = best;
  }
  path.poses = opt;
}

void RRTConnectSmoothPlanner::smoothPath(nav_msgs::msg::Path & path, double target_spacing) {
  if (path.poses.size() < 2) return;
  std::vector<geometry_msgs::msg::PoseStamped> res;
  for (size_t i = 0; i < path.poses.size() - 1; ++i) {
    auto p1 = path.poses[i].pose.position;
    auto p2 = path.poses[i+1].pose.position;
    double d = hypot(p2.x - p1.x, p2.y - p1.y);
    res.push_back(path.poses[i]);
    int steps = std::max(static_cast<int>(d / target_spacing), 1);
    for (int j = 1; j < steps; ++j) {
      double t = static_cast<double>(j) / steps;
      geometry_msgs::msg::PoseStamped p = path.poses[i];
      p.pose.position.x = p1.x + t * (p2.x - p1.x);
      p.pose.position.y = p1.y + t * (p2.y - p1.y);
      res.push_back(p);
    }
  }
  res.push_back(path.poses.back());
  path.poses = res;
}

bool RRTConnectSmoothPlanner::isCollisionFree(const Node & n1, const Node & n2) {
  double d = getDistance(n1, n2);
  // 使用你代码中的 0.05 步长
  int steps = std::max(static_cast<int>(d / 0.05), 1);
  for (int i = 0; i <= steps; ++i) {
    double t = static_cast<double>(i) / steps;
    unsigned int mx, my;
    if (!costmap_->worldToMap(n1.x + t*(n2.x-n1.x), n1.y + t*(n2.y-n1.y), mx, my)) return false;
    if (costmap_->getCost(mx, my) >= 253) return false;
  }
  return true;
}

double RRTConnectSmoothPlanner::getDistance(const Node & n1, const Node & n2) {
  return hypot(n1.x - n2.x, n1.y - n2.y);
}

void RRTConnectSmoothPlanner::publishTree(const std::vector<Node> & start_tree, const std::vector<Node> & goal_tree, const std::string & frame_id) {
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id; m.header.stamp = rclcpp::Clock().now();
  m.ns = "rrt_debug"; m.id = 0; m.type = visualization_msgs::msg::Marker::LINE_LIST;
  m.action = visualization_msgs::msg::Marker::ADD; m.scale.x = 0.015;
  m.pose.orientation.w = 1.0;
  auto add = [&](const std::vector<Node>& t, float r, float g, float b) {
    for (size_t i = 1; i < t.size(); ++i) {
      int p = t[i].parent_idx; if (p == -1) continue;
      geometry_msgs::msg::Point p1, p2;
      p1.x = t[i].x; p1.y = t[i].y; p1.z = 0.02;
      p2.x = t[p].x; p2.y = t[p].y; p2.z = 0.02;
      m.points.push_back(p1); m.points.push_back(p2);
      std_msgs::msg::ColorRGBA c; c.r = r; c.g = g; c.b = b; c.a = 0.5;
      m.colors.push_back(c); m.colors.push_back(c);
    }
  };
  add(start_tree, 0.0, 1.0, 1.0); add(goal_tree, 1.0, 0.0, 0.0);
  marker_pub_->publish(m);
}

} // namespace nav2_rrt_connect_smooth_planner

PLUGINLIB_EXPORT_CLASS(nav2_rrt_connect_smooth_planner::RRTConnectSmoothPlanner, nav2_core::GlobalPlanner)