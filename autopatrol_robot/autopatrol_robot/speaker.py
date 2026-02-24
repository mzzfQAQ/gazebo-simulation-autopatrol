# 需求：编写一个ROS2节点（Python）
# 流程：
#     1.导包
#     2.初始化ROS2客户端
#     3.自定义节点类
#     4.调用spin函数，传入自定义类对象
#     5.释放资源

# 1.导包
import rclpy
from rclpy.node import Node
from autopatrol_interfaces.srv import SpeechText
import espeakng

# 3.自定义节点类
class Speaker(Node):
    def __init__(self):
        super().__init__("speaker_node_py")
        self.get_logger().info("speaker_node_py 节点已启动（python）!")
        self.speech_service_ = self.create_service(SpeechText,'speech_text',self.speech_text_callback)
        self.speaker_ = espeakng.Speaker()
        self.speaker_.voice = 'zh'

    def speech_text_callback(self,request,response):
        self.get_logger().info(f'正在准备阅读{request.text}')
        self.speaker_.say(request.text)
        self.speaker_.wait()
        response.result = True
        return response

def main():
    # 2.初始化ROS2客户端
    rclpy.init()
    # 4.调用spin函数，传入自定义类对象
    rclpy.spin(Speaker())
    # 5.释放资源
    rclpy.shutdown()

if __name__ == '__main__':
    main()