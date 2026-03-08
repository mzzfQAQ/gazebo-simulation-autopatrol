#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <random>
#include <algorithm>

#include "nav2_core/exceptions.hpp"
#include "nav2_custom_planner/nav2_rrt_origin_planner.hpp"

namespace nav2_rrt_origin_planner
{
    // 定义 RRT 树节点结构
    struct Node {
        double x, y;
        int parent_idx;
        Node(double x_, double y_, int p_) : x(x_), y(y_), parent_idx(p_) {}
    };

    void RRTOriginPlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        tf_ = tf;
        node_ = parent.lock();
        name_ = name;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();
        // 参数初始化
        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
        node_->get_parameter(name_ + ".interpolation_resolution",
                             interpolation_resolution_);
    }

    void RRTOriginPlanner::cleanup()
    {
        RCLCPP_INFO(node_->get_logger(), "正在清理类型为 RRTOriginPlanner 的插件 %s",
                    name_.c_str());
    }

    void RRTOriginPlanner::activate()
    {
        RCLCPP_INFO(node_->get_logger(), "正在激活类型为 RRTOriginPlanner 的插件 %s",
                    name_.c_str());
    }

    void RRTOriginPlanner::deactivate()
    {
        RCLCPP_INFO(node_->get_logger(), "正在停用类型为 RRTOriginPlanner 的插件 %s",   
                    name_.c_str());
    }

    nav_msgs::msg::Path RRTOriginPlanner::createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal)
    {
        nav_msgs::msg::Path global_path;
        global_path.header.stamp = node_->now();
        global_path.header.frame_id = global_frame_;

        // --- RRT 参数设置 ---
        double step_size = 0.2;         // 每次扩展的步长
        int max_iterations = 5000;      // 最大迭代次数
        double goal_tolerance = 0.3;    // 距离目标多近算到达
        
        // 获取地图边界用于采样
        double min_x = costmap_->getOriginX();
        double min_y = costmap_->getOriginY();
        double max_x = min_x + costmap_->getSizeInMetersX();
        double max_y = min_y + costmap_->getSizeInMetersY();

        // 随机数生成器
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis_x(min_x, max_x);
        std::uniform_real_distribution<> dis_y(min_y, max_y);

        // 初始化树，起点作为根节点
        std::vector<Node> tree;
        tree.emplace_back(start.pose.position.x, start.pose.position.y, -1);

        bool found_goal = false;
        int last_node_idx = -1;

        // --- 主循环 ---
        for (int i = 0; i < max_iterations; ++i) {
            // 1. 随机采样点 (Random Sample)
            double rnd_x = dis_x(gen);
            double rnd_y = dis_y(gen);

            // 2. 找到树中距离采样点最近的节点 (Nearest Neighbor)
            int nearest_idx = 0;
            double min_dist = std::hypot(tree[0].x - rnd_x, tree[0].y - rnd_y);
            for (size_t j = 1; j < tree.size(); ++j) {
                double d = std::hypot(tree[j].x - rnd_x, tree[j].y - rnd_y);
                if (d < min_dist) {
                    min_dist = d;
                    nearest_idx = j;
                }
            }

            // 3. 向采样点方向扩展固定步长 (Extend/Step)
            double theta = std::atan2(rnd_y - tree[nearest_idx].y, rnd_x - tree[nearest_idx].x);
            double new_x = tree[nearest_idx].x + step_size * std::cos(theta);
            double new_y = tree[nearest_idx].y + step_size * std::sin(theta);

            // ================= 新增/修改：4. 连线碰撞检测 (Edge Collision Check) =================
            bool collision = false;
            // 计算连线的实际距离
            double dist = std::hypot(new_x - tree[nearest_idx].x, new_y - tree[nearest_idx].y);
            // 根据代价地图的分辨率，决定要在这条线上检查多少个点
            int check_steps = std::ceil(dist / costmap_->getResolution()); 
            
            for (int k = 0; k <= check_steps; ++k) {
                // 沿着连线进行插值采样
                double check_x = tree[nearest_idx].x + k * (new_x - tree[nearest_idx].x) / check_steps;
                double check_y = tree[nearest_idx].y + k * (new_y - tree[nearest_idx].y) / check_steps;
                
                unsigned int mx, my;
                if (costmap_->worldToMap(check_x, check_y, mx, my)) {
                    // 核心细节：253 代表小车内切圆膨胀边界，254 代表实体障碍物
                    // 如果 >= 253，说明碰到墙或者碰到墙的膨胀层了（容易卡住）
                    if (costmap_->getCost(mx, my) >= 253) { 
                        collision = true;
                        break; // 只要线上有一个点撞了，这条线就废了
                    }
                } else {
                    collision = true; // 出界了也算撞
                    break;
                }
            }

            if (collision) {
                continue; // 发生碰撞，抛弃这个采样点，进入下一次循环
            }
            // ==============================================================================

            // 5. 添加新节点到树中
            tree.emplace_back(new_x, new_y, nearest_idx);
            
            // 6. 检查是否接近目标
            double dist_to_goal = std::hypot(new_x - goal.pose.position.x, new_y - goal.pose.position.y);
            if (dist_to_goal < goal_tolerance) {
                found_goal = true;
                last_node_idx = tree.size() - 1;
                break;
            }
        }

        // --- 回溯路径 ---
        if (found_goal) {
            int current_idx = last_node_idx;
            while (current_idx != -1) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = global_path.header;
                pose.pose.position.x = tree[current_idx].x;
                pose.pose.position.y = tree[current_idx].y;
                global_path.poses.push_back(pose);
                current_idx = tree[current_idx].parent_idx;
            }
            // RRT回溯是从终点到起点，需要反转
            std::reverse(global_path.poses.begin(), global_path.poses.end());
            
            // 确保最后一个点是精确的目标点
            geometry_msgs::msg::PoseStamped final_goal = goal;
            final_goal.header = global_path.header; // 强制覆盖掉 RViz 带过来的未来时间戳
            global_path.poses.push_back(final_goal);
            return global_path;
        } else {
            RCLCPP_WARN(node_->get_logger(), "RRT 未能找到有效路径");
            return global_path;
        }
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_rrt_origin_planner::RRTOriginPlanner,
                       nav2_core::GlobalPlanner)