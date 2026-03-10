#ifndef NAV2_CUSTOM_PLANNER__NAV2_RRT_ORIGIN_PLANNER_HPP_
#define NAV2_CUSTOM_PLANNER__NAV2_RRT_ORIGIN_PLANNER_HPP_
#include <memory>
#include <string>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" //位置和姿态
#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp" //全局规划器接口 基类 base_type
#include "nav2_costmap_2d/costmap_2d_ros.hpp"  //代价地图ROS接口
#include "nav2_util/lifecycle_node.hpp" //生命周期节点 rclcpp的一个子类
#include "nav2_util/robot_utils.hpp" //机器人工具类
#include "nav_msgs/msg/path.hpp" //路径消息 
#include "visualization_msgs/msg/marker.hpp"//可视化标记消息
#include <chrono> // 用于计时

namespace nav2_rrt_origin_planner {
// RRT原始规划器类
class RRTOriginPlanner : public nav2_core::GlobalPlanner {
public:
    RRTOriginPlanner() = default;
    ~RRTOriginPlanner() = default;
    // 插件配置方法
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
    // 插件清理方法
    void cleanup() override;
    // 插件激活方法
    void activate() override;
    // 插件停用方法
    void deactivate() override;
    void configure(...);
    // 为给定的起始和目标位姿创建路径的方法
    nav_msgs::msg::Path
    createPlan(const geometry_msgs::msg::PoseStamped &start,
               const geometry_msgs::msg::PoseStamped &goal) override;

private:
    // 坐标变换缓存指针，可用于查询坐标关系
    std::shared_ptr<tf2_ros::Buffer> tf_;
    // 节点指针
    nav2_util::LifecycleNode::SharedPtr node_;
    // 全局代价地图
    nav2_costmap_2d::Costmap2D *costmap_;
    // 全局代价地图的坐标系world
    std::string global_frame_, name_;
    // 插值分辨率
    double interpolation_resolution_;

protected:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

} // namespace nav2_rrt_origin_planner

#endif // NAV2_CUSTOM_PLANNER__NAV2_RRT_ORIGIN_PLANNER_HPP_