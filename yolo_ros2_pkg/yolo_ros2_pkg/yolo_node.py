#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        
        # 初始化 CvBridge
        self.bridge = CvBridge()
        
        # 加载 YOLOv8 模型 (首次运行会自动下载 yolov8n.pt 权重文件)
        self.get_logger().info("正在加载 YOLOv8 模型...")
        self.model = YOLO('yolov8n.pt') 
        self.get_logger().info("YOLOv8 模型加载完成！")

        # 订阅 Gazebo 的相机话题 (!!! 请将下面的字符串替换为你实际的相机话题名称 !!!)
        self.camera_topic = '/camera_sensor/image_raw' 
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10)
            
        # 创建发布者，发布带有检测框的图像
        self.publisher = self.create_publisher(Image, '/yolo/annotated_image', 10)

    def image_callback(self, msg):
        try:
            # 1. 将 ROS 2 的 Image 消息转换为 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 2. 使用 YOLOv8 进行推理
            # verbose=False 可以减少终端的冗余打印
            results = self.model(cv_image, verbose=False) 
            
            # 3. 获取带有检测框的渲染图像
            annotated_frame = results[0].plot()
            
            # 4. 将 OpenCV 图像转回 ROS 2 Image 消息并发布
            img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            self.publisher.publish(img_msg)
            
        except Exception as e:
            self.get_logger().error(f"处理图像时发生错误: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()