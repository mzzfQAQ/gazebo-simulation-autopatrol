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
        RCLCPP_INFO(node_->get_logger(), "Cleaning up RRTDynamicBiasedPlanner");
    }

    void RRTDynamicBiasedPlanner::activate()
    {
        RCLCPP_INFO(node_->get_logger(), "Activating RRTDynamicBiasedPlanner");
    }

    void RRTDynamicBiasedPlanner::deactivate()
    {
        RCLCPP_INFO(node_->get_logger(), "Deactivating RRTDynamicBiasedPlanner");
    }

    nav_msgs::msg::Path RRTDynamicBiasedPlanner::createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal)
    {
        auto start_time = std::chrono::steady_clock::now();

        nav_msgs::msg::Path global_path;
        global_path.header.stamp = node_->now();
        global_path.header.frame_id = global_frame_;

        // --- 参数设置 ---
        const double step_size = 0.25;      // 稍微加大步长提升效率
        const int max_iterations = 3000;    // 动态偏置通常不需要5000次即可收敛
        const double goal_tolerance = 0.3;
        
        // 距离敏感型偏置参数
        const double p_base = 0.1;          // 基础偏置 (10% 概率看向目标)
        const double p_close = 0.4;         // 接近目标时的强偏置
        const double proximity_threshold = 2.0; // 进入目标2米范围内触发强偏置

        double min_x = costmap_->getOriginX();
        double min_y = costmap_->getOriginY();
        double max_x = min_x + costmap_->getSizeInMetersX();
        double max_y = min_y + costmap_->getSizeInMetersY();

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis_x(min_x, max_x);
        std::uniform_real_distribution<> dis_y(min_y, max_y);
        std::uniform_real_distribution<> dis_prob(0.0, 1.0);

        std::vector<Node> tree;
        tree.reserve(max_iterations); // 预分配内存避免 push_back 时的多次拷贝
        tree.emplace_back(start.pose.position.x, start.pose.position.y, -1);

        // 可视化 Marker 预设
        visualization_msgs::msg::Marker tree_marker;
        tree_marker.header = global_path.header;
        tree_marker.ns = "rrt_tree";
        tree_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        tree_marker.action = visualization_msgs::msg::Marker::ADD;
        tree_marker.scale.x = 0.02;
        tree_marker.color.r = 0.0; tree_marker.color.g = 0.8; tree_marker.color.b = 1.0; tree_marker.color.a = 0.5;
        tree_marker.points.reserve(max_iterations * 2);

        bool found_goal = false;
        int last_node_idx = -1;

        for (int i = 0; i < max_iterations; ++i) {
            // 1. 获取当前树中最后一个点到目标的距离，决定采样策略
            // 这里用最近邻可能更准确，但直接用 current_dist 逻辑更简单且有效
            double dist_to_goal_nearest = std::hypot(tree.back().x - goal.pose.position.x, 
                                                     tree.back().y - goal.pose.position.y);
            
            // 动态决定 p_bias: 离得越近，越想直接连目标
            double p_bias = (dist_to_goal_nearest < proximity_threshold) ? p_close : p_base;

            double rnd_x, rnd_y;
            if (dis_prob(gen) < p_bias) {
                rnd_x = goal.pose.position.x;
                rnd_y = goal.pose.position.y;
            } else {
                rnd_x = dis_x(gen);
                rnd_y = dis_y(gen);
            }

            // 2. 寻找最近节点 (O(n) - 建议后续升级为 KD-Tree)
            int nearest_idx = 0;
            double min_dist_sq = std::numeric_limits<double>::max();
            for (size_t j = 0; j < tree.size(); ++j) {
                double dx = tree[j].x - rnd_x;
                double dy = tree[j].y - rnd_y;
                double d_sq = dx*dx + dy*dy; // 使用平方比较减少 sqrt 计算
                if (d_sq < min_dist_sq) {
                    min_dist_sq = d_sq;
                    nearest_idx = j;
                }
            }

            // 3. 步进扩展
            double angle = std::atan2(rnd_y - tree[nearest_idx].y, rnd_x - tree[nearest_idx].x);
            double new_x = tree[nearest_idx].x + step_size * std::cos(angle);
            double new_y = tree[nearest_idx].y + step_size * std::sin(angle);

            // 4. 碰撞检测
            bool collision = false;
            unsigned int mx, my;
            // 检查终点是否在地图内及其代价
            if (!costmap_->worldToMap(new_x, new_y, mx, my) || costmap_->getCost(mx, my) >= 253) {
                collision = true;
            } else {
                // 线性插值检测路径连线是否安全
                double res = costmap_->getResolution();
                int steps = std::max(static_cast<int>(step_size / res), 1);
                for (int k = 1; k < steps; ++k) {
                    double check_x = tree[nearest_idx].x + (new_x - tree[nearest_idx].x) * k / steps;
                    double check_y = tree[nearest_idx].y + (new_y - tree[nearest_idx].y) * k / steps;
                    unsigned int cmx, cmy;
                    if (!costmap_->worldToMap(check_x, check_y, cmx, cmy) || costmap_->getCost(cmx, cmy) >= 253) {
                        collision = true;
                        break;
                    }
                }
            }

            if (collision) continue;

            // 5. 添加节点
            tree.emplace_back(new_x, new_y, nearest_idx);
            
            // 记录可视化
            geometry_msgs::msg::Point p1, p2;
            p1.x = tree[nearest_idx].x; p1.y = tree[nearest_idx].y; p1.z = 0.05;
            p2.x = new_x; p2.y = new_y; p2.z = 0.05;
            tree_marker.points.push_back(p1);
            tree_marker.points.push_back(p2);

            // 6. 检查是否到达目标
            double final_dist = std::hypot(new_x - goal.pose.position.x, new_y - goal.pose.position.y);
            if (final_dist < goal_tolerance) {
                found_goal = true;
                last_node_idx = tree.size() - 1;
                break;
            }
        }

        marker_pub_->publish(tree_marker);

        if (found_goal) {
            int current = last_node_idx;
            while (current != -1) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = global_path.header;
                pose.pose.position.x = tree[current].x;
                pose.pose.position.y = tree[current].y;
                pose.pose.orientation.w = 1.0;
                global_path.poses.push_back(pose);
                current = tree[current].parent_idx;
            }
            std::reverse(global_path.poses.begin(), global_path.poses.end());
            
            // 确保最后一个点是精确的目标点
            geometry_msgs::msg::PoseStamped goal_pose = goal;
            goal_pose.header = global_path.header;
            global_path.poses.push_back(goal_pose);
        }

        auto end_time = std::chrono::steady_clock::now();
        double ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        RCLCPP_INFO(node_->get_logger(), "Planner: %s | 耗时: %.2f ms | 采样节点数: %zu", 
                    found_goal ? "Success" : "Failed", ms, tree.size());

        return global_path;
    }
}

PLUGINLIB_EXPORT_CLASS(nav2_rrt_dynamic_biased_planner::RRTDynamicBiasedPlanner, nav2_core::GlobalPlanner)