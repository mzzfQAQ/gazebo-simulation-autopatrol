#include "nav2_custom_planner/nav2_rrt_dynamic_biased_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <random>
#include <algorithm>
#include <chrono>

#include "nav2_core/exceptions.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_rrt_dynamic_biased_planner
{
    struct Node {
        double x, y;
        int parent_idx;
        Node(double x_, double y_, int p_) : x(x_), y(y_), parent_idx(p_) {}
    };

    void RRTDynamicBiasedPlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        tf_ = tf;
        node_ = parent.lock();
        name_ = name;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
        node_->get_parameter(name_ + ".interpolation_resolution",
                             interpolation_resolution_);

        marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("rrt_tree_markers", 10);
    }

    void RRTDynamicBiasedPlanner::cleanup()
    {
        RCLCPP_INFO(node_->get_logger(), "正在清理类型为 RRTDynamicBiasedPlanner 的插件 %s", name_.c_str());
    }

    void RRTDynamicBiasedPlanner::activate()
    {
        RCLCPP_INFO(node_->get_logger(), "正在激活类型为 RRTDynamicBiasedPlanner 的插件 %s", name_.c_str());
    }

    void RRTDynamicBiasedPlanner::deactivate()
    {
        RCLCPP_INFO(node_->get_logger(), "正在停用类型为 RRTDynamicBiasedPlanner 的插件 %s", name_.c_str());
    }

    nav_msgs::msg::Path RRTDynamicBiasedPlanner::createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal)
    {
        auto start_time = std::chrono::steady_clock::now();

        nav_msgs::msg::Path global_path;
        global_path.header.stamp = node_->now();
        global_path.header.frame_id = global_frame_;

        // --- Marker 可视化初始化 ---
        visualization_msgs::msg::Marker tree_marker;
        tree_marker.header = global_path.header;
        tree_marker.ns = "rrt_tree";
        tree_marker.id = 0;
        tree_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        tree_marker.action = visualization_msgs::msg::Marker::ADD;
        tree_marker.pose.orientation.w = 1.0;
        tree_marker.scale.x = 0.02; 
        tree_marker.color.r = 0.0; tree_marker.color.g = 0.8; tree_marker.color.b = 1.0; 
        tree_marker.color.a = 0.6;

        // --- RRT 算法参数 ---
        double step_size = 0.2;
        int max_iterations = 5000;
        double goal_tolerance = 0.3;
        
        // --- 动态调整核心参数 (对应图示 y = p_min + (p_max - p_min) * f(k/K)) ---
        double p_min = 0.05; // 最小概率 (起始)
        double p_max = 0.45; // 最大概率 (峰值)
        double alpha = 3.0;  // 后期幂函数的指数（决定曲线向上弯曲的程度）

        double min_x = costmap_->getOriginX();
        double min_y = costmap_->getOriginY();
        double max_x = min_x + costmap_->getSizeInMetersX();
        double max_y = min_y + costmap_->getSizeInMetersY();

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis_x(min_x, max_x);
        std::uniform_real_distribution<> dis_y(min_y, max_y);
        std::uniform_real_distribution<> dis_prob(0.0, 1.0); // 概率判定采样

        std::vector<Node> tree;
        tree.emplace_back(start.pose.position.x, start.pose.position.y, -1);

        bool found_goal = false;
        int last_node_idx = -1;

        // --- 核心迭代循环 ---
        for (int i = 0; i < max_iterations; ++i) {
            double k_over_K = static_cast<double>(i) / static_cast<double>(max_iterations);
            double p_bias = 0.0;

            // 1. 根据图示分段函数动态调整 p_bias
            if (k_over_K < 0.3) {
                // 第一阶段：对数增长 (Blue Section)
                p_bias = p_min + (p_max - p_min) * std::log2(1.0 + k_over_K);
            } 
            else if (k_over_K <= 0.7) {
                // 第二阶段：线性增长 (Green Section)
                p_bias = p_min + (p_max - p_min) * k_over_K;
            } 
            else {
                // 第三阶段：幂函数增长 (Red Section)
                p_bias = p_min + (p_max - p_min) * std::pow(k_over_K, alpha);
            }

            // 2. 动态采样决策
            double rnd_x, rnd_y;
            if (dis_prob(gen) < p_bias) {
                // 目标导向：直接向目标生长
                rnd_x = goal.pose.position.x;
                rnd_y = goal.pose.position.y;
            } else {
                // 自由探索：在地图范围内随机采样
                rnd_x = dis_x(gen);
                rnd_y = dis_y(gen);
            }

            // 3. 寻找最近节点
            int nearest_idx = 0;
            double min_dist = std::hypot(tree[0].x - rnd_x, tree[0].y - rnd_y);
            for (size_t j = 1; j < tree.size(); ++j) {
                double d = std::hypot(tree[j].x - rnd_x, tree[j].y - rnd_y);
                if (d < min_dist) {
                    min_dist = d;
                    nearest_idx = j;
                }
            }

            // 4. 步进扩展
            double theta = std::atan2(rnd_y - tree[nearest_idx].y, rnd_x - tree[nearest_idx].x);
            double new_x = tree[nearest_idx].x + step_size * std::cos(theta);
            double new_y = tree[nearest_idx].y + step_size * std::sin(theta);

            // 5. 碰撞检测逻辑
            bool collision = false;
            double dist = std::hypot(new_x - tree[nearest_idx].x, new_y - tree[nearest_idx].y);
            int check_steps = std::ceil(dist / costmap_->getResolution()); 
            
            for (int k = 0; k <= check_steps; ++k) {
                double check_x = tree[nearest_idx].x + k * (new_x - tree[nearest_idx].x) / check_steps;
                double check_y = tree[nearest_idx].y + k * (new_y - tree[nearest_idx].y) / check_steps;
                
                unsigned int mx, my;
                if (costmap_->worldToMap(check_x, check_y, mx, my)) {
                    if (costmap_->getCost(mx, my) >= 253) { 
                        collision = true;
                        break; 
                    }
                } else {
                    collision = true;
                    break;
                }
            }

            if (collision) continue;

            // 6. 更新树结构
            tree.emplace_back(new_x, new_y, nearest_idx);
            
            // 记录 Marker 线段用于可视化
            geometry_msgs::msg::Point p_start, p_end;
            p_start.x = tree[nearest_idx].x; p_start.y = tree[nearest_idx].y; p_start.z = 0.05;
            p_end.x = new_x; p_end.y = new_y; p_end.z = 0.05;
            tree_marker.points.push_back(p_start);
            tree_marker.points.push_back(p_end);

            // 7. 终点检查
            double dist_to_goal = std::hypot(new_x - goal.pose.position.x, new_y - goal.pose.position.y);
            if (dist_to_goal < goal_tolerance) {
                found_goal = true;
                last_node_idx = tree.size() - 1;
                break;
            }
        }

        // 发布 RVIZ 树可视化
        marker_pub_->publish(tree_marker);

        auto end_time = std::chrono::steady_clock::now();
        double duration = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        RCLCPP_INFO(node_->get_logger(), 
            "动态偏置RRT规划完成 | 采样数: %ld | 耗时: %.2f ms | 结果: %s", 
            tree.size(), duration, found_goal ? "成功" : "失败");

        // --- 路径回溯 ---
        if (found_goal) {
            int current_idx = last_node_idx;
            while (current_idx != -1) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = global_path.header;
                pose.pose.position.x = tree[current_idx].x;
                pose.pose.position.y = tree[current_idx].y;
                pose.pose.orientation.w = 1.0;
                global_path.poses.push_back(pose);
                current_idx = tree[current_idx].parent_idx;
            }
            std::reverse(global_path.poses.begin(), global_path.poses.end());
            
            // 注入最终精确目标点
            geometry_msgs::msg::PoseStamped final_goal = goal;
            final_goal.header = global_path.header;
            global_path.poses.push_back(final_goal);
            return global_path;
        } else {
            RCLCPP_WARN(node_->get_logger(), "RRT 未能在最大迭代次数内找到路径");
            return global_path;
        }
    }
}

// 插件导出
PLUGINLIB_EXPORT_CLASS(nav2_rrt_dynamic_biased_planner::RRTDynamicBiasedPlanner, nav2_core::GlobalPlanner)