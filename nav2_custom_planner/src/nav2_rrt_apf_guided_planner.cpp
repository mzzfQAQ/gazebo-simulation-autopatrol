#include "nav2_custom_planner/nav2_rrt_apf_guided_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <random>
#include <algorithm>
#include <chrono> 
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
        RCLCPP_INFO(node_->get_logger(), "RRT-APF 优化版（高性能距离偏置）已配置。");
    }

    void RRTAPFGuidedPlanner::cleanup() { marker_pub_.reset(); }
    void RRTAPFGuidedPlanner::activate() { marker_pub_->on_activate(); }
    void RRTAPFGuidedPlanner::deactivate() { marker_pub_->on_deactivate(); }

    nav_msgs::msg::Path RRTAPFGuidedPlanner::createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal)
    {
        auto start_time_clock = std::chrono::steady_clock::now();
        
        auto now = node_->now(); 
        nav_msgs::msg::Path global_path;
        global_path.header.stamp = now;
        global_path.header.frame_id = global_frame_;

        visualization_msgs::msg::Marker tree_marker;
        tree_marker.header.stamp = now;
        tree_marker.header.frame_id = global_frame_;
        tree_marker.ns = "apf_rrt";
        tree_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        tree_marker.action = visualization_msgs::msg::Marker::ADD;
        tree_marker.scale.x = 0.015; // 稍微细一点减少渲染压力
        tree_marker.color.r = 0.0; tree_marker.color.g = 1.0; tree_marker.color.b = 0.5; tree_marker.color.a = 0.7;
        tree_marker.pose.orientation.w = 1.0;

        // --- 核心算法参数 (使用 constexpr 提高编译器优化) ---
        const double step_size = 0.25;
        const int max_iter = 3000; 
        const double goal_tol_sq = 0.35 * 0.35; 
        const double eta = 0.3, zeta = 0.15, d0 = 0.6; 
        
        // 目标点缓存
        const double gx = goal.pose.position.x;
        const double gy = goal.pose.position.y;

        // 初始距离计算
        double init_dist_sq = (gx - start.pose.position.x) * (gx - start.pose.position.x) + 
                              (gy - start.pose.position.y) * (gy - start.pose.position.y);
        double current_min_dist_sq = init_dist_sq;

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis_x(costmap_->getOriginX(), costmap_->getOriginX() + costmap_->getSizeInMetersX());
        std::uniform_real_distribution<> dis_y(costmap_->getOriginY(), costmap_->getOriginY() + costmap_->getSizeInMetersY());
        std::uniform_real_distribution<> dis_prob(0.0, 1.0);

        std::vector<Node> tree;
        tree.reserve(max_iter); // 关键优化：预留空间防止 vector resize 耗时
        tree.emplace_back(start.pose.position.x, start.pose.position.y, -1);

        bool found = false;
        for (int i = 0; i < max_iter; ++i) {
            
            // 1. 动态偏置逻辑：基于当前树离目标的最近距离 (避免每轮循环全树扫描)
            // 距离越近，dist_ratio 越小，p_bias 越大
            double dist_ratio = std::sqrt(current_min_dist_sq / init_dist_sq);
            double p_bias = 0.5 - (0.4 * dist_ratio); 

            // 2. 采样
            double rx = (dis_prob(gen) < p_bias) ? gx : dis_x(gen);
            double ry = (dis_prob(gen) < p_bias) ? gy : dis_y(gen);

            // 3. 寻找最近节点 (单次遍历)
            int nearest_idx = 0;
            double min_d_sample_sq = 1e9;
            for (size_t j = 0; j < tree.size(); ++j) {
                double dx = tree[j].x - rx;
                double dy = tree[j].y - ry;
                double d_sq = dx*dx + dy*dy;
                if (d_sq < min_d_sample_sq) {
                    min_d_sample_sq = d_sq;
                    nearest_idx = j;
                }
            }

            // 4. 步进计算
            double angle = std::atan2(ry - tree[nearest_idx].y, rx - tree[nearest_idx].x);
            double nx = tree[nearest_idx].x + step_size * std::cos(angle);
            double ny = tree[nearest_idx].y + step_size * std::sin(angle);

            // 5. 优化的 APF 斥力引导
            double fx = zeta * (gx - nx);
            double fy = zeta * (gy - ny);

            unsigned int mx, my;
            if (costmap_->worldToMap(nx, ny, mx, my)) {
                // 仅当节点靠近潜在障碍物（Cost > 0）时才计算斥力窗口
                if (costmap_->getCost(mx, my) > 0) {
                    int r = 2; // 缩小搜索半径进一步提速
                    for (int ox = -r; ox <= r; ox++) {
                        for (int oy = -r; oy <= r; oy++) {
                            unsigned int cx = mx + ox, cy = my + oy;
                            if (cx < costmap_->getSizeInCellsX() && cy < costmap_->getSizeInCellsY()) {
                                if (costmap_->getCost(cx, cy) >= 253) {
                                    double wx, wy; costmap_->mapToWorld(cx, cy, wx, wy);
                                    double dx = nx - wx;
                                    double dy = ny - wy;
                                    double d_sq = dx*dx + dy*dy;
                                    if (d_sq < d0 * d0 && d_sq > 0.001) {
                                        double d = std::sqrt(d_sq);
                                        double factor = eta * (1.0/d - 1.0/d0) / (d_sq * d);
                                        fx += factor * dx;
                                        fy += factor * dy;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            nx += 0.1 * fx; ny += 0.1 * fy;

            // 6. 碰撞检测
            unsigned int fmx, fmy;
            if (!costmap_->worldToMap(nx, ny, fmx, fmy) || costmap_->getCost(fmx, fmy) >= 253) continue;

            // 7. 添加节点并更新最短距离记录
            tree.emplace_back(nx, ny, nearest_idx);
            
            double d_to_goal_sq = (nx - gx)*(nx - gx) + (ny - gy)*(ny - gy);
            if (d_to_goal_sq < current_min_dist_sq) {
                current_min_dist_sq = d_to_goal_sq;
            }

            // 可视化 (由于 LINE_LIST 耗内存，可以隔几个点加一次或者限制总数)
            if (i % 2 == 0) {
                geometry_msgs::msg::Point p1, p2;
                p1.x = tree[nearest_idx].x; p1.y = tree[nearest_idx].y; p1.z = 0.1;
                p2.x = nx; p2.y = ny; p2.z = 0.1;
                tree_marker.points.push_back(p1);
                tree_marker.points.push_back(p2);
            }

            // 8. 终点判断
            if (d_to_goal_sq < goal_tol_sq) {
                found = true; break;
            }
        }

        marker_pub_->publish(tree_marker);

        auto end_time_clock = std::chrono::steady_clock::now();
        double duration = std::chrono::duration<double, std::milli>(end_time_clock - start_time_clock).count();

        if (found) {
            int curr = static_cast<int>(tree.size()) - 1;
            while (curr != -1) {
                geometry_msgs::msg::PoseStamped p;
                p.header = global_path.header;
                p.pose.position.x = tree[curr].x; p.pose.position.y = tree[curr].y;
                p.pose.orientation.w = 1.0;
                global_path.poses.push_back(p);
                curr = tree[curr].parent_idx;
            }
            std::reverse(global_path.poses.begin(), global_path.poses.end());
            RCLCPP_INFO(node_->get_logger(), "规划成功 | 采样节点数: %zu | 耗时: %.2f ms", tree.size(), duration);
            return global_path;
        }

        RCLCPP_WARN(node_->get_logger(), "规划失败 | 耗时: %.2f ms", duration);
        return global_path;
    }
}

PLUGINLIB_EXPORT_CLASS(nav2_rrt_apf_guided_planner::RRTAPFGuidedPlanner, nav2_core::GlobalPlanner)