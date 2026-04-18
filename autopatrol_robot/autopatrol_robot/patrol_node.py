import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import TransformListener, Buffer
from rclpy.duration import Duration
from autopatrol_interfaces.srv import SpeechText
from sensor_msgs.msg import Image
from std_msgs.msg import Bool # 接收检测信号
from cv_bridge import CvBridge  
import cv2 
import time
import math # 【新增】引入原生的 math 库进行四元数计算

class PatrolNode(BasicNavigator):
    def __init__(self, node_name='patrol_node'):
        super().__init__(node_name)
        self.declare_parameter('initial_point', [0.0, 0.0, 0.0])
        # 按照 [x, y, yaw] 的顺序，规划 4 个巡检目标点 (默认参数，会被 yaml 覆盖)
        self.declare_parameter('target_points', [
             8.0, -2.5,  3.14,  # 目标点1：左上病房，朝左
            -3.0, -2.8,  0.0,   # 目标点2：右上病房，朝右
            -7.5,  2.0, -1.57,  # 目标点3：右下病房，朝下
             0.0,  0.0,  1.57   # 目标点4：返回中央大厅，朝上
        ])
        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value
        
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)
        self.speach_client_ = self.create_client(SpeechText, 'speech_text')

        self.declare_parameter('image_save_path', '') 
        self.image_save_path = self.get_parameter('image_save_path').value
        self.bridge = CvBridge()
        self.latest_image = None
        
        # 订阅 YOLO 画好框的图像
        self.subscription_image = self.create_subscription(
            Image, '/yolo/annotated_image', self.image_callback, 10)
        
        # 订阅 YOLO 的检测状态，并设置状态标志位
        self.subscription_detect = self.create_subscription(
            Bool, '/yolo/person_detected', self.detect_callback, 10)
        self.person_detected = False
        self.is_interacting = False # 是否正在与老人交互（防止重复触发）
        self.last_interaction_time = 0.0

    def image_callback(self, msg):
        self.latest_image = msg

    def detect_callback(self, msg):
        self.person_detected = msg.data

    def record_image(self, prefix="normal"):
        if self.latest_image is not None:
            pose = self.get_current_pose()
            if pose is not None:
                cv_image = self.bridge.imgmsg_to_cv2(self.latest_image)
                filename = f'{self.image_save_path}{prefix}_image_{pose.translation.x:3.2f}_{pose.translation.y:3.2f}.png'
                cv2.imwrite(filename, cv_image)
                self.get_logger().info(f'图像已保存: {filename}')

    def speach_text(self, text):
        while not self.speach_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('语音服务未上线，等待中。。。')
        request = SpeechText.Request()
        request.text = text
        future = self.speach_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().result:
            self.get_logger().info(f'语音: {text}')

    def get_pose_by_xyyaw(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        
        # 【修改】将传入的 yaw 强制转为浮点数
        yaw_float = float(yaw)
        self.get_logger().info(f"\n\n ====> 正在前往目标: X={x}, Y={y}, 设定的终点朝向(Yaw)={yaw_float} <==== \n")
        
        # 【修改】手动计算偏航角对应的四元数 (仅绕 Z 轴旋转)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw_float / 2.0)
        pose.pose.orientation.w = math.cos(yaw_float / 2.0)
        return pose

    def init_robot_pose(self):
        self.setInitialPose(self.get_pose_by_xyyaw(
            self.initial_point_[0], self.initial_point_[1], self.initial_point_[2]))
        self.waitUntilNav2Active()

    def get_target_points(self):
        points = []
        for index in range(int(len(self.target_points_)/3)):
            x = self.target_points_[index*3]
            y = self.target_points_[index*3+1]
            yaw = self.target_points_[index*3+2]
            points.append([x, y, yaw])
        return points

    def get_current_pose(self):
        try:
            tf = self.buffer_.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            return tf.transform
        except Exception:
            return None

    def nav_to_pose_with_perception(self, target_pose):
        """带视觉感知中断的导航核心逻辑"""
        self.goToPose(target_pose)
        
        while not self.isTaskComplete():
            # 1. 检查冷却时间是否结束
            current_time = time.time()
            if self.is_interacting and (current_time - self.last_interaction_time > 150):
                self.is_interacting = False
                self.get_logger().info('冷却结束，恢复对人员的感知。')

            # 2. 如果看到人，且当前没有在交互，且冷却时间已过
            if self.person_detected and not self.is_interacting:
                self.get_logger().warn('!!! 紧急中断：发现看护对象 !!!')
                self.cancelTask() # 取消当前 Nav2 导航任务
                
                # 开始交互流程
                self.is_interacting = True
                self.last_interaction_time = time.time()
                
                self.speach_text(text='您好，检测到看护对象，正在记录当前安全状态。')
                self.record_image(prefix="care_target")
                self.speach_text(text='记录完成，祝您身体健康，我将继续巡逻。')
                
                return False # 返回 False 代表是被打断的，没有到达目标点
                
            rclpy.spin_once(self, timeout_sec=0.1) # 保证回调函数能刷新检测标志位
            
        result = self.getResult()
        return result == TaskResult.SUCCEEDED

def main():
    rclpy.init()
    patrol = PatrolNode()
    patrol.speach_text(text='康养巡检系统启动，正在初始化位置')
    patrol.init_robot_pose()
    
    points = patrol.get_target_points()
    point_idx = 0
    
    # 使用状态机循环，保证被打断后还能继续去未完成的目标点
    while rclpy.ok() and point_idx < len(points):
        x, y, yaw = points[point_idx]
        target_pose = patrol.get_pose_by_xyyaw(x, y, yaw)
        
        patrol.speach_text(text=f'准备前往第{point_idx+1}个病房巡检点')
        
        # 导航并捕获结果
        reached_target = patrol.nav_to_pose_with_perception(target_pose)
        
        if reached_target:
            patrol.speach_text(text=f"已到达病房{point_idx+1}，环境正常")
            patrol.record_image(prefix="room_env")
            
            # 【关键修改】：到达巡检点后，强制清除冷却状态，立即恢复对人员的感知
            if patrol.is_interacting:
                patrol.is_interacting = False
                patrol.get_logger().info('已到达巡检点，重置冷却时间，立即恢复人员感知。')
                
            point_idx += 1 # 只有真正到达了，才切换到下一个点
            
        else:
            # 如果是 False，说明是半路遇到老人被打断了，重新发起去这个点的任务
            patrol.get_logger().info('导航被打断，处理完突发事件后恢复原定路线...')
            time.sleep(2) # 缓冲一下

    patrol.speach_text(text='所有康养巡检任务执行完毕。')
    rclpy.shutdown()

if __name__ == '__main__':
    main()