import rclpy
from rclpy.node import Node
from ros2_yolo_msgs.msg import DetectedBox

class BoxPublisher(Node):
    def __init__(self):
        super().__init__('test_box_publisher')
        self.publisher = self.create_publisher(DetectedBox, 'detected_boxes', 10)

        # 创建定时器，发送一次消息
        self.timer = self.create_timer(1.0, self.publish_msg)
        self.sent = False

    def publish_msg(self):
        if self.sent:
            return

        msg = DetectedBox()
        msg.x1 = 114.0
        msg.y1 = 514.0
        msg.x2 = 191.0
        msg.y2 = 810.0
        msg.servo = 0

        self.publisher.publish(msg)
        self.get_logger().info('已发布测试消息: [%.1f, %.1f] → [%.1f, %.1f], servo=%d' %
                               (msg.x1, msg.y1, msg.x2, msg.y2, msg.servo))
        self.sent = True

def main(args=None):
    rclpy.init(args=args)
    node = BoxPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
