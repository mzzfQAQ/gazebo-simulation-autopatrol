#include "nav2_custom_planner/nav2_rrt_apf_guided_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <random>
#include <algorithm>
#include <chrono> // 必须包含计时库
#include "pluginlib/class_list_macros.hpp"

namespace nav2_rrt_apf_guided_planner
{
    void RRTAPFGuidedPlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        node_ = parent.lock();
        name_ = name;
        tf_ = tf;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();

        marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("rrt_tree_markers", 10);
        RCLCPP_INFO(node_->get_logger(), "RRTAPFGuidedPlanner 插件已配置。");
    }

    void RRTAPFGuidedPlanner::cleanup() { marker_pub_.reset(); }
    void RRTAPFGuidedPlanner::activate() { marker_pub_->on_activate(); }
    void RRTAPFGuidedPlanner::deactivate() { marker_pub_->on_deactivate(); }

    nav_msgs::msg::Path RRTAPFGuidedPlanner::createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal)
    {
        // --- 开始计时 ---
        auto start_time_clock = std::chrono::steady_clock::now();
        
        auto now = node_->now(); 
        nav_msgs::msg::Path global_path;
        global_path.header.stamp = now;
        global_path.header.frame_id = global_frame_;

        visualization_msgs::msg::Marker tree_marker;
        tree_marker.header.stamp = now;
        tree_marker.header.frame_id = global_frame_;
        tree_marker.ns = "apf_rrt";
        tree_marker.id = 0;
        tree_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        tree_marker.action = visualization_msgs::msg::Marker::ADD;
        tree_marker.scale.x = 0.02;
        tree_marker.color.r = 1.0; tree_marker.color.g = 0.5; tree_marker.color.a = 0.9;
        tree_marker.pose.orientation.w = 1.0;

        // 核心算法参数
        double step_size = 0.25;
        int max_iter = 3000; 
        double goal_tol = 0.35;
        double eta = 0.3, zeta = 0.15, d0 = 0.6; 

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis_x(costmap_->getOriginX(), costmap_->getOriginX() + costmap_->getSizeInMetersX());
        std::uniform_real_distribution<> dis_y(costmap_->getOriginY(), costmap_->getOriginY() + costmap_->getSizeInMetersY());
        std::uniform_real_distribution<> dis_prob(0.0, 1.0);

        std::vector<Node> tree;
        tree.emplace_back(start.pose.position.x, start.pose.position.y, -1);

        bool found = false;
        for (int i = 0; i < max_iter; ++i) {
            double ratio = static_cast<double>(i) / max_iter;
            double p_bias = (ratio < 0.3) ? 0.1 + 0.3 * std::log2(1.0 + ratio) : 0.45;

            double rx = (dis_prob(gen) < p_bias) ? goal.pose.position.x : dis_x(gen);
            double ry = (dis_prob(gen) < p_bias) ? goal.pose.position.y : dis_y(gen);

            int n_idx = 0;
            double m_dist = 1e9;
            for (size_t j = 0; j < tree.size(); ++j) {
                double d = std::hypot(tree[j].x - rx, tree[j].y - ry);
                if (d < m_dist) { m_dist = d; n_idx = j; }
            }

            double angle = std::atan2(ry - tree[n_idx].y, rx - tree[n_idx].x);
            double nx = tree[n_idx].x + step_size * std::cos(angle);
            double ny = tree[n_idx].y + step_size * std::sin(angle);

            // APF 斥力引导
            double fx = zeta * (goal.pose.position.x - nx);
            double fy = zeta * (goal.pose.position.y - ny);

            unsigned int mx, my;
            if (costmap_->worldToMap(nx, ny, mx, my)) {
                int r = 3; 
                for (int ox = -r; ox <= r; ox++) {
                    for (int oy = -r; oy <= r; oy++) {
                        unsigned int cx = mx + ox, cy = my + oy;
                        if (cx < costmap_->getSizeInCellsX() && cy < costmap_->getSizeInCellsY()) {
                            if (costmap_->getCost(cx, cy) >= 253) {
                                double wx, wy; costmap_->mapToWorld(cx, cy, wx, wy);
                                double d = std::hypot(nx - wx, ny - wy);
                                if (d < d0 && d > 0.01) {
                                    fx += eta * (1.0/d - 1.0/d0) * (nx - wx) / (d*d*d);
                                    fy += eta * (1.0/d - 1.0/d0) * (ny - wy) / (d*d*d);
                                }
                            }
                        }
                    }
                }
            }
            nx += 0.1 * fx; ny += 0.1 * fy;

            unsigned int fmx, fmy;
            if (!costmap_->worldToMap(nx, ny, fmx, fmy) || costmap_->getCost(fmx, fmy) >= 253) continue;

            tree.emplace_back(nx, ny, n_idx);
            
            geometry_msgs::msg::Point p1, p2;
            p1.x = tree[n_idx].x; p1.y = tree[n_idx].y; p1.z = 0.15;
            p2.x = nx; p2.y = ny; p2.z = 0.15;
            tree_marker.points.push_back(p1);
            tree_marker.points.push_back(p2);

            if (std::hypot(nx - goal.pose.position.x, ny - goal.pose.position.y) < goal_tol) {
                found = true; break;
            }
        }

        marker_pub_->publish(tree_marker);

        // --- 计算总耗时 ---
        auto end_time_clock = std::chrono::steady_clock::now();
        double duration = std::chrono::duration<double, std::milli>(end_time_clock - start_time_clock).count();

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

            // --- 适配统计脚本的单行日志输出 ---
            RCLCPP_INFO(node_->get_logger(), 
                "RRT 规划结束 | 采样节点数: %ld | 耗时: %.2f ms | 结果: 成功", 
                tree.size(), duration);
            
            return global_path;
        }

        RCLCPP_WARN(node_->get_logger(), 
            "RRT 规划结束 | 采样节点数: %ld | 耗时: %.2f ms | 结果: 失败", 
            tree.size(), duration);

        return global_path;
    }
}

PLUGINLIB_EXPORT_CLASS(nav2_rrt_apf_guided_planner::RRTAPFGuidedPlanner, nav2_core::GlobalPlanner)