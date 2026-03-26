#include "nav2_custom_planner/nav2_rrt_connect_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_rrt_connect_planner
{

void RRTConnectPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  name_ = name;

  // 初始化参数
  nav2_util::declare_parameter_if_not_declared(node, name + ".step_size", rclcpp::ParameterValue(0.2));
  node->get_parameter(name + ".step_size", step_size_);
  
  // 初始化 Marker 发布器
  marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("rrt_tree_markers", 10);
}

void RRTConnectPlanner::cleanup() { marker_pub_.reset(); }
void RRTConnectPlanner::activate() {  }
void RRTConnectPlanner::deactivate() {  }

nav_msgs::msg::Path RRTConnectPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;
  global_path.header.stamp = rclcpp::Clock().now();
  global_path.header.frame_id = start.header.frame_id;

  std::vector<Node> start_tree, goal_tree;
  start_tree.push_back({start.pose.position.x, start.pose.position.y, -1});
  goal_tree.push_back({goal.pose.position.x, goal.pose.position.y, -1});

  std::default_random_engine gen((std::random_device())());
  std::uniform_real_distribution<double> dis_x(costmap_->getOriginX(), costmap_->getOriginX() + costmap_->getSizeInMetersX());
  std::uniform_real_distribution<double> dis_y(costmap_->getOriginY(), costmap_->getOriginY() + costmap_->getSizeInMetersY());

  bool connected = false;
  int start_node_final_idx = -1;
  int goal_node_final_idx = -1;
  bool start_is_A = true; // 用于追踪哪棵树是起点树

  for (int i = 0; i < max_iterations_; ++i) {
    Node rand_node = {dis_x(gen), dis_y(gen), -1};
    Node new_node_current;

    // 尝试扩展当前的树 A
    if (extendTree(start_tree, rand_node, new_node_current)) {
      // 尝试让树 B 向树 A 的新节点连接
      Node new_node_other;
      if (extendTree(goal_tree, new_node_current, new_node_other)) {
        if (getDistance(new_node_current, new_node_other) < connection_threshold_) {
          connected = true;
          start_node_final_idx = start_is_A ? (start_tree.size() - 1) : (goal_tree.size() - 1);
          goal_node_final_idx = start_is_A ? (goal_tree.size() - 1) : (start_tree.size() - 1);
          break;
        }
      }
    }
    // 交换树以平衡扩展
    std::swap(start_tree, goal_tree);
    start_is_A = !start_is_A;
  }

  // 可视化随机树 (无论是否成功都发布，方便调试)
  publishTree(start_is_A ? start_tree : goal_tree, start_is_A ? goal_tree : start_tree, start.header.frame_id);

  if (connected) {
    // 此时我们要确保拿到正确的 tree 引用来回溯
    // 逻辑简化：根据 start_is_A 的状态找到真正的 start_tree 和 goal_tree
    std::vector<Node>& real_start_tree = start_is_A ? start_tree : goal_tree;
    std::vector<Node>& real_goal_tree = start_is_A ? goal_tree : start_tree;

    // 回溯起点树
    int curr = start_node_final_idx;
    while (curr != -1) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = real_start_tree[curr].x;
      pose.pose.position.y = real_start_tree[curr].y;
      global_path.poses.push_back(pose);
      curr = real_start_tree[curr].parent_idx;
    }
    std::reverse(global_path.poses.begin(), global_path.poses.end());

    // 拼接终点树
    curr = goal_node_final_idx;
    while (curr != -1) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = real_goal_tree[curr].x;
      pose.pose.position.y = real_goal_tree[curr].y;
      global_path.poses.push_back(pose);
      curr = real_goal_tree[curr].parent_idx;
    }
  }

  return global_path;
}

bool RRTConnectPlanner::extendTree(std::vector<Node> & tree, const Node & target, Node & new_node) {
  int nearest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < tree.size(); ++i) {
    double d = getDistance(tree[i], target);
    if (d < min_dist) { min_dist = d; nearest_idx = i; }
  }

  double theta = atan2(target.y - tree[nearest_idx].y, target.x - tree[nearest_idx].x);
  new_node.x = tree[nearest_idx].x + step_size_ * cos(theta);
  new_node.y = tree[nearest_idx].y + step_size_ * sin(theta);
  new_node.parent_idx = nearest_idx;

  if (isCollisionFree(tree[nearest_idx], new_node)) {
    tree.push_back(new_node);
    return true;
  }
  return false;
}

double RRTConnectPlanner::getDistance(const Node & n1, const Node & n2) {
  return hypot(n1.x - n2.x, n1.y - n2.y);
}

bool RRTConnectPlanner::isCollisionFree(const Node & n1, const Node & n2) {
  double dist = getDistance(n1, n2);
  int steps = std::max(static_cast<int>(dist / 0.05), 1);
  for (int i = 0; i <= steps; ++i) {
    double x = n1.x + (n2.x - n1.x) * i / steps;
    double y = n1.y + (n2.y - n1.y) * i / steps;
    unsigned int mx, my;
    if (!costmap_->worldToMap(x, y, mx, my)) return false;
    unsigned char cost = costmap_->getCost(mx, my);
    if (cost >= 253) return false; 
  }
  return true;
}

void RRTConnectPlanner::publishTree(const std::vector<Node> & start_tree, const std::vector<Node> & goal_tree, const std::string & frame_id) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = "rrt_trees";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.015; // 线宽
  
  auto add_to_marker = [&](const std::vector<Node>& tree, float r, float g, float b) {
    for (size_t i = 1; i < tree.size(); ++i) {
      int p_idx = tree[i].parent_idx;
      if (p_idx == -1) continue;
      geometry_msgs::msg::Point p_start, p_end;
      p_start.x = tree[i].x; p_start.y = tree[i].y; p_start.z = 0.1;
      p_end.x = tree[p_idx].x; p_end.y = tree[p_idx].y; p_end.z = 0.1;
      marker.points.push_back(p_start);
      marker.points.push_back(p_end);
      std_msgs::msg::ColorRGBA color;
      color.r = r; color.g = g; color.b = b; color.a = 0.6;
      marker.colors.push_back(color);
      marker.colors.push_back(color);
    }
  };

  add_to_marker(start_tree, 0.0, 0.5, 1.0); // 起点树-蓝色
  add_to_marker(goal_tree, 1.0, 0.0, 0.0);  // 终点树-红色
  marker_pub_->publish(marker);
}

} // namespace nav2_rrt_connect_planner

PLUGINLIB_EXPORT_CLASS(nav2_rrt_connect_planner::RRTConnectPlanner, nav2_core::GlobalPlanner)