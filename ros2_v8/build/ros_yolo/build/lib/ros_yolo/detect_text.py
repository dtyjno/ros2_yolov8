from ros2_yolo_msgs.msg import DetectedBox
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import threading
import time

class AIDetector(Node):
    """智能检测节点（从ROS 2图像话题接收图像并检测）"""

    def __init__(self):
        super().__init__('ai_detector')

        # ========== 参数声明 ==========
        self.declare_parameters(
            namespace='',
            parameters=[
                ('image_topic', 'image_topic'),  # 图像订阅话题
                #('image_topic', 'raw_images'),  # 图像订阅话题
                ('model_path1', 'best_old_circle.pt'),  # 模型路径1
                ('model_path2', 'best_H.pt'),# 模型路径2
                ('conf_threshold', 0.5),  # 置信度阈值
                ('device', 'cuda:0'),  # 推理设备
                ('frame_size', [1920, 1080]),  # [宽, 高]
            ]
        )

        # ========== 组件初始化 ==========
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.running = True

        self._init_model()
        self._init_subscribers()
        self._init_publishers()
        self._init_threads()
        self.center_x, self.center_y = 640, 360  # 圆心位置
        self.radius = 40  # 半径
        self.inside_counter = 0  # 连续帧计数
        self.inside_threshold = 15  # 连续帧阈值
        self.pause_until = None  # 控制发布暂停时间
        self.get_logger().info("节点初始化完成")

    def _init_model(self):
        """初始化AI模型"""
        device = self.get_parameter('device').value
        try:
            model_path1 = self.get_parameter('model_path1').value
            model_path2 = self.get_parameter('model_path2').value

            self.model1 = YOLO(model_path1).to(device)
            self.model2 = YOLO(model_path2).to(device)
            self.model1.fuse()
            self.model2.fuse()

            self.get_logger().info(f"已加载模型1: {model_path1} → {device}")
            self.get_logger().info(f"已加载模型2: {model_path2} → {device}")
        except Exception as e:
            self.get_logger().error(f"模型加载失败: {str(e)}")
            raise

    def _init_subscribers(self):
        """初始化图像订阅器"""
        image_topic = self.get_parameter('image_topic').value
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos
        )

    def _init_publishers(self):
        """初始化检测框发布器"""
        self.box_pub = self.create_publisher(DetectedBox, 'detected_boxes', 10)

    def _init_threads(self):
        """启动处理线程"""
        self.proc_thread = threading.Thread(target=self._process_loop)
        self.proc_thread.start()

    def image_callback(self, msg):
        """图像话题回调"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.frame_lock:
                self.latest_frame = frame
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {str(e)}")

    def _process_loop(self):
        """AI处理循环（双模型推理 + 合并检测结果）"""
        while self.running:
            start_time = time.time()

            # 获取最新帧
            with self.frame_lock:
                frame = self.latest_frame.copy() if self.latest_frame is not None else None

            if frame is None:
                time.sleep(0.001)
                continue

            # 推理参数
            conf_thres = self.get_parameter('conf_threshold').value

            try:
                # 分别执行两个模型推理
                results1 = self.model1.predict(source=frame, conf=conf_thres, verbose=False)
                results2 = self.model2.predict(source=frame, conf=conf_thres, verbose=False)

                # 合并两个模型的推理结果
                combined_results = results1 + results2

                # 绘制检测框并发布最大目标
                annotated_frame = self._draw_detections(frame, combined_results)

                # 显示处理图像
                window_width, window_height = 1920, 1080
                resized_frame = cv2.resize(annotated_frame, (window_width, window_height))
                cv2.imshow('text_Detection', resized_frame)
                cv2.waitKey(1)

                # 打印性能日志
                self.get_logger().info(
                    f"处理延迟: {(time.time() - start_time) * 1000:.1f}ms",
                    throttle_duration_sec=0.33
                )
            except Exception as e:
                self.get_logger().error(f"推理过程出错: {str(e)}")
                time.sleep(0.05)


    def _draw_detections(self, frame, results):
        """绘制检测结果并发布最大目标（含圆内判断+servo控制）"""
        cv2.circle(frame, (self.center_x, self.center_y), self.radius, (0, 255, 255), 2, cv2.LINE_AA)

        target_boxes = []
        largest_box = None

        for result in results:
            for box in result.boxes.cpu().numpy():
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_id = int(box.cls[0])
                label_name = result.names[cls_id]
                label = f"{label_name} {box.conf[0]:.2f}"

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                if label_name != 'person':
                    area = (x2 - x1) * (y2 - y1)
                    target_boxes.append((area, x1, y1, x2, y2))

        # 控制发布逻辑
        if target_boxes and (self.pause_until is None or time.time() >= self.pause_until):
            largest_box = max(target_boxes, key=lambda b: b[0])
            _, x1, y1, x2, y2 = largest_box
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2

            # 判断是否在圆内
            dx = center_x - self.center_x
            dy = center_y - self.center_y
            in_circle = dx * dx + dy * dy <= self.radius * self.radius

            if in_circle:
                self.inside_counter += 1
            else:
                self.inside_counter = 0

            # 构建并发布消息
            msg = DetectedBox()
            msg.x1 = float(x1)
            msg.y1 = float(y1)
            msg.x2 = float(x2)
            msg.y2 = float(y2)

            if self.inside_counter >= self.inside_threshold:
                msg.servo = 1
                self.box_pub.publish(msg)
                self.get_logger().info("目标在圆内15帧，servo=1 已发布")
                self.pause_until = time.time() + 0.5  # 暂停0.5秒
                self.inside_counter = 0  # 重置计数器
            else:
                msg.servo = 0
                self.box_pub.publish(msg)

        return frame

    def destroy_node(self):
        """节点关闭处理"""
        self.running = False
        self.proc_thread.join()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AIDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
