#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class PersonPatrolNode(Node):
    def __init__(self):
        super().__init__('person_patrol_node')
        
        # 创建发布者，话题名必须与 world 文件中的 remapping 保持一致
        self.publisher_ = self.create_publisher(Twist, '/person_cmd_vel', 10)
        
        # 创建一个定时器，每 0.1 秒执行一次控制逻辑
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 状态机初始化
        self.state = 'WALK'
        self.state_start_time = time.time()
        
        # 运动参数设置
        self.walk_speed = 0.2      # 走路速度 0.2 m/s
        self.walk_duration = 30.0  # 每次直走 30 秒 (即单程 3 米)
        self.turn_speed = 1.0      # 转身角速度 1.0 rad/s
        self.turn_duration = 3.14  # 转身持续时间 3.14 秒 (刚好转 180 度)

        self.get_logger().info('🚶 行人巡逻节点已启动！')

    def timer_callback(self):
        msg = Twist()
        current_time = time.time()
        elapsed_time = current_time - self.state_start_time

        if self.state == 'WALK':
            msg.linear.y = -self.walk_speed
            msg.angular.z = 0.0
            
            # 如果直走时间够了，切换到转身状态
            if elapsed_time >= self.walk_duration:
                self.state = 'TURN'
                self.state_start_time = time.time()
                self.get_logger().info('到达巡逻边界，开始转身...')

        elif self.state == 'TURN':
            msg.linear.y = 0.0
            msg.angular.z = self.turn_speed
            
            # 如果转身时间够了，切换回直走状态
            if elapsed_time >= self.turn_duration:
                self.state = 'WALK'
                self.state_start_time = time.time()
                self.get_logger().info('转身完毕，继续直线巡逻...')

        # 发布速度指令
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PersonPatrolNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('巡逻被手动终止')
    finally:
        # 退出前发送一个停止指令，防止人一直往前滑
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()