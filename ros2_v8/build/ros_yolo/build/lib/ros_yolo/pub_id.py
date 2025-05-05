import rclpy
from rclpy.node import Node
from ros2_yolo_msgs.msg import DetectedBox


class CoordinateListener(Node):
    """监听检测框坐标并显示在终端"""

    def __init__(self):
        super().__init__('coordinate_listener')

        # 创建订阅器，订阅检测框坐标
        self.subscription = self.create_subscription(
            DetectedBox,
            'detected_boxes',
            self.listener_callback,
            10
        )
        self.get_logger().info('开始监听检测框坐标...')

    def listener_callback(self, msg):
        """回调函数，处理接收到的坐标数据并显示在终端"""
        self.get_logger().info(
            f"检测框坐标: ({msg.x1}, {msg.y1}), ({msg.x2}, {msg.y2})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CoordinateListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
