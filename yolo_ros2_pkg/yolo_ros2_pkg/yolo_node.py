#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool  # 引入 Bool 消息，用于发布是否看到人
from cv_bridge import CvBridge
from ultralytics import YOLO

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        self.bridge = CvBridge()
        self.get_logger().info("正在加载 YOLOv8 模型...")
        self.model = YOLO('yolov8n.pt') 
        self.get_logger().info("YOLOv8 模型加载完成！")

        self.camera_topic = '/camera_sensor/image_raw' 
        self.subscription = self.create_subscription(Image, self.camera_topic, self.image_callback, 10)
            
        self.img_publisher = self.create_publisher(Image, '/yolo/annotated_image', 10)
        # 【新增】发布目标检测状态的 Publisher
        self.detect_publisher = self.create_publisher(Bool, '/yolo/person_detected', 10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(cv_image, verbose=False) 
            annotated_frame = results[0].plot()
            
            # 【新增逻辑】遍历检测结果，看看有没有 'person' (类别ID通常为0)
            person_found = False
            for box in results[0].boxes:
                if int(box.cls[0]) == 0:  # COCO 数据集中 0 代表 person
                    person_found = True
                    break
            
            # 发布检测状态
            msg_bool = Bool()
            msg_bool.data = person_found
            self.detect_publisher.publish(msg_bool)

            img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            self.img_publisher.publish(img_msg)
            
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