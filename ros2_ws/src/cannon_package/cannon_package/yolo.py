#!/usr/bin/env python3
import sys
sys.path.append('/home/pivi/libcamera/build/src/py')
sys.path.append('/home/pivi/kmsxx/build/py')

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion, Vector3
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
from picamera2 import Picamera2
from threading import Lock, Thread

DEFAULT_CONFIDENCE = 0.7
DEFAULT_IMGSZ = 640

_frame_lock = Lock()
latest_frame = None

def set_frame(frame):
    global latest_frame
    with _frame_lock:
        latest_frame = frame.copy()

def get_frame():
    with _frame_lock:
        return None if latest_frame is None else latest_frame.copy()

class YoloPublisher(Node):
    def __init__(self, model_path: str):
        super().__init__('yolo_publisher')
        self.get_logger().info('Initializing YOLO Node...')

        # Declare parameters
        self.declare_parameter('model_path', model_path)
        self.declare_parameter('confidence', DEFAULT_CONFIDENCE)
        self.declare_parameter('imgsz', DEFAULT_IMGSZ)

        path = self.get_parameter('model_path').value
        self.confidence = self.get_parameter('confidence').value
        self.imgsz = self.get_parameter('imgsz').value

        # Subscription and publishers in a Reentrant group
        group = ReentrantCallbackGroup()
        # self.subscription = self.create_subscription(
        #     Image, '/image_raw', self.image_callback, 1,
        #     callback_group=group
        # )
        self.bbox_publisher = self.create_publisher(
            Vector3, '/bbox', 100
        )
        self.image_publisher = self.create_publisher(
            Image, 'yolo/image_raw', 100
        )

        # YOLO setup
        self.bridge = CvBridge()
        self.model = YOLO(path, task='detect')
        self.get_logger().info(f'Loaded YOLO model from {path}')

        # PiCamera2 setup
        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={"size": (640, 360), "format": "RGB888"},
            controls={"FrameRate": 120.0}
        )
        self.picam2.configure(config)
        self.picam2.start()

        self.get_logger().info('YOLO Node Start Detecting...')

        # Timer for pulling frames (non-blocking)
        self.timer = self.create_timer(
            0.1, self.timer_callback, callback_group=group
        )

    def timer_callback(self):
        try:
            frame = self.picam2.capture_array()
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            results = self.model(frame, conf=self.confidence, imgsz=self.imgsz, verbose=True)
            annotated = results[0].plot()
            set_frame(annotated)

            boxes = results[0].boxes
            if not boxes:
                center_xy = Vector3(x=0.0, y=100.0, z=0.0)
                self.bbox_publisher.publish(center_xy)
                return

            x1, y1, x2, y2 = map(float, boxes[0].xyxy[0])
            center_xy = Vector3(x=(x1+x2)/2, y=100.0, z=0.0)
            self.bbox_publisher.publish(center_xy)

            msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            self.image_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error in timer_callback: {e}')

    # def image_callback(self, msg: Image):
    #     try:
    #         image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    #         results = self.model(source=image, conf=self.confidence, imgsz=self.imgsz)
    #         annotated = results[0].plot()
    #         set_frame(annotated)

    #         if self.height is None or self.width is None:
    #             self.height = msg.height
    #             self.width = msg.width
    #             self.get_logger().info(
    #                 f'Set dimensions: h={self.height}, w={self.width}'
    #             )

    #         boxes = results[0].boxes
    #         if not boxes:
    #             return

    #         x1, y1, x2, y2 = map(float, boxes[0].xyxy[0])
    #         q = Quaternion(x=x1, y=y1, z=x2 - x1, w=y2 - y1)
    #         self.original_publisher.publish(q)

    #         center = (x1 + x2) / 2
    #         norm_x = (center / self.width) * 2 - 1
    #         v = Vector3(x=norm_x, y=0.0, z=0.0)
    #         self.normalized_publisher.publish(v)
    #     except Exception as e:
    #         self.get_logger().error(f'Error in image_callback: {e}')

    def destroy_node(self):
        # Stop camera on shutdown
        try:
            self.picam2.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloPublisher(model_path='yolov8n.pt')
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()