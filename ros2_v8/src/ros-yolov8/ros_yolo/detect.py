# src/ros2-yolov8/ros_yolo/detect_v8.py
from ros2_yolo_msgs.msg import DetectedBox
from std_msgs.msg import Int32
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
    """智能检测节点（含摄像头直读+原始图像发布）"""

    def __init__(self):
        super().__init__('ai_detector')

        # ========== 参数声明 ==========
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_id', '0'),  # 摄像头ID/URL
                ('model_path1', '/home/ak47k98/PycharmProjects/ros2_v8/best_circle.pt'),  # 模型路径1
                ('model_path2', '/home/ak47k98/PycharmProjects/ros2_v8/best_H.pt'),# 模型路径2
                ('conf_threshold', 0.6),  # 置信度阈值
                ('device', 'cuda:0'),  # 推理设备
                ('frame_size', [1920, 1080]),  # [宽, 高]
                ('publish_raw', True),  # 是否发布原始图像
                # ('rtsp_url', '')               # RTSP预留接口
            ]
        )

        # ========== 硬件初始化 ==========
        self._init_camera()
        self.bridge = CvBridge()

        # ========== 模型初始化 ==========
        self._init_model()

        # ========== ROS接口 ==========
        self._init_publishers()
        self._init_threads()
        # 圆心相关初始化（建议加在 __init__ 的末尾）
        self.center_x, self.center_y = 640, 360  # 圆心位置
        self.radius = 40  # 半径
        self.inside_counter = 0  # 连续帧计数
        self.inside_threshold = 30  # 连续帧阈值
        self.pause_until = None  # 控制发布暂停时间

        self.current_state = 0  # 默认状态
        self.state_sub = self.create_subscription(
            Int32,
            'current_state',
            self._state_callback,
            10
        )


        self.get_logger().info("节点初始化完成")

    def _init_camera(self):
        """初始化视频采集设备"""
        camera_id = self.get_parameter('camera_id').value
        self.cap = cv2.VideoCapture(camera_id)

        if not self.cap.isOpened():
            self.get_logger().error(f"无法打开摄像头: {camera_id}")
            raise RuntimeError("Camera open failed")

        # 设置采集参数
        frame_size = self.get_parameter('frame_size').value
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_size[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_size[1])
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 最小化缓冲区

    def _state_callback(self, msg: Int32):
        self.current_state = msg.data
        self.get_logger().info(f"接收到状态更新: {self.current_state}", throttle_duration_sec=2.0)

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

    def _init_publishers(self):
        """初始化发布器"""
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # 原始图像发布器
        if self.get_parameter('publish_raw').value:
            self.raw_pub = self.create_publisher(Image, 'raw_images', qos)
        else:
            self.raw_pub = None
        self.box_pub = self.create_publisher(
            DetectedBox,
            'detected_boxes',
            10
        )
    def _init_threads(self):
        """启动处理线程"""
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.running = True

        # 图像采集线程
        self.cap_thread = threading.Thread(target=self._capture_loop)
        self.cap_thread.start()

        # AI处理线程
        self.proc_thread = threading.Thread(target=self._process_loop)
        self.proc_thread.start()

    def _capture_loop(self):
        """图像采集循环"""
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("视频帧获取失败", throttle_duration_sec=1)
                continue

            with self.frame_lock:
                self.latest_frame = frame

            # 发布原始图像
            if self.raw_pub:
                try:
                    msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    self.raw_pub.publish(msg)
                except Exception as e:
                    self.get_logger().error(f"图像发布失败: {str(e)}")

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
                cv2.imshow('AI Detection', resized_frame)
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
        """绘制检测结果，分开发布 circle 和 H 坐标，servo=1 只受 circle 控制"""
        cv2.circle(frame, (self.center_x, self.center_y), self.radius, (0, 255, 255), 2, cv2.LINE_AA)
        # 如果状态是 3，先做 stuffed 弹窗逻辑
        if self.current_state == 3:     #侦查
            idx = 0
            for result in results:
                for box in result.boxes.cpu().numpy():
                    cls_id = int(box.cls[0])
                    label_name = result.names[cls_id]
                    if label_name != 'H':
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        roi = frame[y1:y2, x1:x2]
                        if roi.size == 0:
                            continue
                        win = f'Stuffed_{idx}'
                        resized_roi = cv2.resize(roi, (160, 160))
                        cv2.imshow(win, resized_roi)
                        cv2.moveWindow(win, 50 + idx * 180, 50)  # 每个窗口水平间隔180px
                        idx += 1
            return frame
        elif self.current_state != 1:
            # 分别存储两类目标框  常规逻辑
            circle_boxes = []
            h_boxes = []

            for result in results:
                for box in result.boxes.cpu().numpy():
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls_id = int(box.cls[0])
                    label_name = result.names[cls_id]
                    conf = float(box.conf[0])

                    if label_name != 'person':
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(frame, f"{label_name} {conf:.2f}", (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                        area = (x2 - x1) * (y2 - y1)
                        if label_name == 'circle':
                            circle_boxes.append((area, x1, y1, x2, y2))
                        elif label_name == 'H':
                            h_boxes.append((area, x1, y1, x2, y2))

            # 初始化消息
            msg = DetectedBox()
            msg.servo = 0

            # ----- 处理 circle 区域检测与 servo -----
            if circle_boxes and (self.pause_until is None or time.time() >= self.pause_until):
                _, x1, y1, x2, y2 = max(circle_boxes, key=lambda b: b[0])
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2

                msg.x1 = float(cx)
                msg.y1 = float(cy)

                # 判断是否在目标圆内
                dx = cx - self.center_x
                dy = cy - self.center_y
                if dx * dx + dy * dy <= self.radius * self.radius and self.current_state == 0:
                    self.inside_counter += 1
                elif self.current_state == 0 :
                    self.inside_counter = 0

                if self.inside_counter >= self.inside_threshold and self.current_state == 0:  # 投弹发布
                    msg.servo = 1
                    self.pause_until = time.time() + 1.0
                    self.inside_counter = 0
                    self.get_logger().info("circle 连续30帧在圆内，servo=1 已发布")

            # ----- 处理 H 坐标 -----
            if h_boxes:
                _, x1, y1, x2, y2 = max(h_boxes, key=lambda b: b[0])
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2

                msg.x2 = float(cx)
                msg.y2 = float(cy)

            # 发布消息（只发一条，包含 circle 和 H 的中心坐标）
            self.box_pub.publish(msg)
            return frame
        else :
            return frame





    def destroy_node(self):
        """节点关闭处理"""
        self.running = False
        self.cap_thread.join()
        self.proc_thread.join()
        self.cap.release()
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
