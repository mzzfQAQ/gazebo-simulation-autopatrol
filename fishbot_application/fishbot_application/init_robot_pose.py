# 需求：编写一个发布机器人初始位置
# 流程：
#     1.导包
#     2.初始化ROS2客户端
#     3.实例化导航器对象，它会自动创建一个隐藏的 ROS 节点来与 Nav2 堆栈通信
#     4.创建一个 PoseStamped 对象，用于设置机器人在地图上的初始位姿
#     5.将上面定义的位姿发送给 Nav2，告知系统机器人在地图上的起始位置（类似 RViz 中的 2D Pose Estimate）
#     6.阻塞式等待，直到 Nav2 的所有子系统（如生命周期管理器）都进入激活状态
#     7.调用spin使程序运行
#     8.释放资源


# 1.导包
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator


def main():
    # 2.初始化ROS2客户端
    rclpy.init()
    # 3.实例化导航器对象，它会自动创建一个隐藏的 ROS 节点来与 Nav2 堆栈通信
    nav = BasicNavigator()
    # 4.创建一个 PoseStamped 对象，用于设置机器人在地图上的初始位姿
    init_pose = PoseStamped()
    init_pose.header.frame_id = "map"
    init_pose.header.stamp = nav.get_clock().now().to_msg()
    init_pose.pose.position.x = 0.0
    init_pose.pose.position.y = 0.0
    init_pose.pose.orientation.w = 1.0
    # 5.将上面定义的位姿发送给 Nav2，告知系统机器人在地图上的起始位置（类似 RViz 中的 2D Pose Estimate）
    nav.setInitialPose(init_pose)
    # 6.阻塞式等待，直到 Nav2 的所有子系统（如生命周期管理器）都进入激活状态
    nav.waitUntilNav2Active()
    # 7.调用spin使程序运行
    rclpy.spin(nav)
    # 8.释放资源
    rclpy.shutdown()




if __name__ == '__main__':
    main()