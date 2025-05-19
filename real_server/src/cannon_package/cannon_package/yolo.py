#!/usr/bin/env python3



# import sys
# sys.path.append('/home/pivi/libcamera/build/src/py')
# sys.path.append('/home/pivi/kmsxx/build/py')

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Vector3
# from cv_bridge import CvBridge
# from ultralytics import YOLO
# import cv2
# from picamera2 import Picamera2
# from libcamera import Transform
# from threading import Lock, Thread
# import math

# # shared storage for the latest JPEG-encoded frame
# _frame_lock = Lock()
# latest_frame = None

# def set_frame(jpg_bytes: bytes):
#     global latest_frame
#     with _frame_lock:
#         latest_frame = jpg_bytes

# def get_frame() -> bytes | None:
#     with _frame_lock:
#         if latest_frame is None:
#             print("[get_frame] No frame available")  # 🔍 Add this
#         return latest_frame

# class YoloPublisher(Node):
#     def __init__(self, model_path: str):
#         super().__init__('yolo_publisher')
#         self.get_logger().info('Initializing YOLO Node...')

#         # parameters
#         self.declare_parameter('model_path', model_path)
#         self.declare_parameter('confidence', 0.7)
#         self.declare_parameter('imgsz', 640)
#         self.model_path = self.get_parameter('model_path').value
#         self.confidence = self.get_parameter('confidence').value
#         self.imgsz = self.get_parameter('imgsz').value

#         # ROS publisher for bounding boxes (optional)
#         self.bbox_pub = self.create_publisher(Vector3, 'bbox', 10)
#         # from sensor_msgs.msg import Image # remove this later
#         # self.image_publisher = self.create_publisher( # remove this later
#         #     Image, 'yolo/image_raw', 1 # remove this later
#         # ) # remove this later

#         # OpenCV bridge & YOLO model
#         self.bridge = CvBridge()
#         self.model = YOLO(self.model_path, task='detect')
#         self.get_logger().info(f'Loaded YOLO model from {self.model_path}')

#         # PiCamera2 setup
#         info = Picamera2.global_camera_info()
#         if not info:
#             self.get_logger().error(f"No cameras found! global_camera_info returned: {info}")
#             raise RuntimeError("No camera devices detected by Picamera2")

#         self.picam2 = Picamera2()
#         config = self.picam2.create_video_configuration(
#             main={"size": (640, 480), "format": "RGB888"},
#             controls={"FrameRate": 30.0},
#             # transform=Transform(vflip=True, hflip=True) # remove this later
#         )
#         self.picam2.configure(config)
#         self.picam2.start()

#         # # start detection thread
#         # self._stop_flag = False
#         # self._detect_thread = Thread(target=self._detection_loop, daemon=True)
#         # self._detect_thread.start()
#         self.timer = self.create_timer(0.2, self._detection_loop)

#     def _detection_loop(self):
#         try:
#             self.get_logger().info("[YOLO] Capturing frame...")
#             frame = self.picam2.capture_array()
#             if frame is None:
#                 self.get_logger().warn("Captured frame is None")
#                 return

#             # inference
#             results = self.model(frame, conf=self.confidence, imgsz=self.imgsz)
#             annotated = results[0].plot()

#             # draw red cross (horizontal and vertical lines through center)
#             height, width = annotated.shape[:2]
#             center_x = width // 2
#             center_y = height // 2
#             cv2.line(annotated, (0, center_y), (width, center_y), (0, 0, 255), 1)  # horizontal red line
#             cv2.line(annotated, (center_x, 0), (center_x, height), (0, 0, 255), 1)  # vertical red line

#             # publish bounding box center
#             boxes = results[0].boxes
#             if not boxes:
#                 center = Vector3(x=math.nan, y=math.nan, z=math.nan)
#             else:
#                 x1, y1, x2, y2 = map(float, boxes[0].xyxyn[0])
#                 cx = (x1 + x2) / 2
#                 cy = (y1 + y2) / 2
#                 center = Vector3(x=2*cx-1, y=-2*cy+1, z=0.0)
#             self.bbox_pub.publish(center)

#             # msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8') # remove this later
#             # self.image_publisher.publish(msg) # remove this later
#             # JPEG-encode once
#             ret, jpg = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
#             if not ret:
#                 self.get_logger().warn("[YOLO] JPEG encoding failed")
#             set_frame(jpg.tobytes())
#         except Exception as e:
#             self.get_logger().error(f'Detection loop error: {e}')

#     def destroy_node(self):
#         # self._stop_flag = True
#         # self._detect_thread.join(timeout=1)
#         try:
#             self.picam2.stop()
#         except Exception:
#             pass
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = YoloPublisher(model_path='yolov8n.pt')
#     try:
#         rclpy.spin(node)
#     finally:
#         node.get_logger().info('Shutting down YOLO node...')
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()




# ORIGINAL

# import sys
# sys.path.append('/home/pivi/libcamera/build/src/py')
# sys.path.append('/home/pivi/kmsxx/build/py')

# import rclpy
# from rclpy.node import Node
# from rclpy.callback_groups import ReentrantCallbackGroup
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Quaternion, Vector3
# from cv_bridge import CvBridge
# from ultralytics import YOLO
# import cv2
# from picamera2 import Picamera2
# from libcamera import Transform
# from threading import Lock, Thread
# import os
# import numpy as np
# import glob

# DEFAULT_CONFIDENCE = 0.7
# DEFAULT_IMGSZ = 640

# _frame_lock = Lock()
# latest_frame = None

# def set_frame(frame):
#     global latest_frame
#     with _frame_lock:
#         latest_frame = frame.copy()

# def get_frame():
#     with _frame_lock:
#         return None if latest_frame is None else latest_frame.copy()

# class YoloPublisher(Node):
#     def __init__(self, model_path: str):
#         super().__init__('yolo_publisher')
#         self.get_logger().info('Initializing YOLO Node...')

#         # Declare parameters
#         self.declare_parameter('model_path', model_path)
#         self.declare_parameter('confidence', DEFAULT_CONFIDENCE)
#         self.declare_parameter('imgsz', DEFAULT_IMGSZ)

#         path = self.get_parameter('model_path').value
#         self.confidence = self.get_parameter('confidence').value
#         self.imgsz = self.get_parameter('imgsz').value

#         # Subscription and publishers in a Reentrant group
#         group = ReentrantCallbackGroup()
#         # self.subscription = self.create_subscription(
#         #     Image, '/image_raw', self.image_callback, 1,
#         #     callback_group=group
#         # )
#         self.bbox_publisher = self.create_publisher(
#             Quaternion, 'bbox', 100
#         )
#         self.image_publisher = self.create_publisher(
#             Image, 'yolo/image_raw', 100
#         )

#         # YOLO setup
#         self.bridge = CvBridge()
#         self.model = YOLO(path, task='detect')
#         self.get_logger().info(f'Loaded YOLO model from {path}')

#         # PiCamera2 setup
#         input_imgsz = (640, 360)
#         self.picam2 = Picamera2()
#         config = self.picam2.create_video_configuration(
#             main={"size": (input_imgsz[0], input_imgsz[1]), "format": "RGB888"},
#             controls={"FrameRate": 30.0},
#             # transform=Transform(vflip=True, hflip=True)
#         )
#         self.picam2.configure(config)
#         self.picam2.start()

#         self.get_logger().info('YOLO Node Start Detecting...')

#         # Timer for pulling frames (non-blocking)
#         self.timer = self.create_timer(
#             0.2, self.timer_callback, callback_group=group
#         )

#         # Supplementary folder for saving undetected images
#         self.folder = "/home/pivi/sandbox/undetected"
#         os.makedirs(self.folder, exist_ok=True)
#         pattern = os.path.join(self.folder, '*.jpg')
#         self.count = len(glob.glob(pattern))

#     def timer_callback(self):
#         try:
#             frame = self.picam2.capture_array()
#             # uncomment this when camera is installed upside down (when cable is on top of camera)
#             # same thing can be done in camera configuration using libcamera's tranform
#             # frame = cv2.rotate(frame, cv2.ROTATE_180)
#             frame = np.ascontiguousarray(frame)
#             results = self.model(frame, conf=self.confidence, imgsz=self.imgsz,)
#             # set_frame(frame)

#             boxes = results[0].boxes
#             if not boxes:
#                 # Send NaN values when no object is detected and save the image for future model training
#                 bbox = Quaternion(x=float('nan'), y=float('nan'), z=float('nan'), w=float('nan'))
#                 # cv2.imwrite(os.path.join(self.folder, f"{self.count}.jpg"), frame)
#                 self.count += 1
#             else:
#                 # Get coordinates in x1,y1,x2,y2 order
#                 x1, y1, x2, y2 = map(float, boxes[0].xyxyn[0])
#                 bbox = Quaternion(x=x1, y=y1, z=x2, w=y2)  # x1,y1,x2,y2 order
#             self.bbox_publisher.publish(bbox)

#             # annotated = results[0].plot()
#             # msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
#             # self.image_publisher.publish(msg)

#         except Exception as e:
#             self.get_logger().error(f'Error in timer_callback: {e}')

#     # def image_callback(self, msg: Image):
#     #     try:
#     #         image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#     #         results = self.model(source=image, conf=self.confidence, imgsz=self.imgsz)
#     #         annotated = results[0].plot()
#     #         set_frame(annotated)

#     #         if self.height is None or self.width is None:
#     #             self.height = msg.height
#     #             self.width = msg.width
#     #             self.get_logger().info(
#     #                 f'Set dimensions: h={self.height}, w={self.width}'
#     #             )

#     #         boxes = results[0].boxes
#     #         if not boxes:
#     #             return

#     #         x1, y1, x2, y2 = map(float, boxes[0].xyxy[0])
#     #         q = Quaternion(x=x1, y=y1, z=x2 - x1, w=y2 - y1)
#     #         self.original_publisher.publish(q)

#     #         center = (x1 + x2) / 2
#     #         norm_x = (center / self.width) * 2 - 1
#     #         v = Vector3(x=norm_x, y=0.0, z=0.0)
#     #         self.normalized_publisher.publish(v)
#     #     except Exception as e:
#     #         self.get_logger().error(f'Error in image_callback: {e}')

#     def destroy_node(self):
#         # Stop camera on shutdown
#         try:
#             self.picam2.stop()
#         except Exception:
#             pass
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = YoloPublisher(model_path='yolov8n.pt')
#     try:
#         rclpy.spin(node)
#     finally:
#         node.get_logger().info('YOLO Node Shutting Down...')
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# AI HAT version

import sys
sys.path.append('/home/pivi/libcamera/build/src/py')
sys.path.append('/home/pivi/kmsxx/build/py')

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion, Vector3
from cv_bridge import CvBridge
import cv2
from picamera2 import Picamera2
from libcamera import Transform
import os
import numpy as np
import glob

# ——— Hailo API imports ———
from hailo_platform import (
    HEF,
    Device,
    VDevice,
    InputVStreamParams,
    OutputVStreamParams,
    FormatType,
    HailoStreamInterface,
    InferVStreams,
    ConfigureParams,
)

DEFAULT_CONFIDENCE = 0.5

# CLASS_NAMES = ["bell"]  # your 80-class list
CLASS_NAMES = [
    "person", "bicycle", "car", "motorbike", "aeroplane",
    "bus", "train", "truck", "boat", "traffic light",
    "fire hydrant", "stop sign", "parking meter", "bench", "bird",
    "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat",
    "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
    "wine glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut",
    "cake", "chair", "sofa", "potted plant", "bed",
    "dining table", "toilet", "tv monitor", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven",
    "toaster", "sink", "refrigerator", "book", "clock",
    "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]
COLORS = np.random.uniform(0, 255, (len(CLASS_NAMES), 3))

def draw_bboxes(image, boxes, scores, classes):
    for (x1, y1, x2, y2), score, cls in zip(boxes, scores, classes):
        x1 = int(640 * x1)
        y1 = int(360 * y1)
        x2 = int(640 * x2)
        y2 = int(360 * y2)
        label = f"{CLASS_NAMES[cls]}:{score:.2f}"
        color = tuple(int(c) for c in COLORS[cls])
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cv2.putText(image, label, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

class YoloPublisher(Node):
    def __init__(self, model_path: str):
        super().__init__('yolo_publisher')
        self.get_logger().info('Initializing HAILO-based YOLO Node...')

        # Declare parameters
        self.declare_parameter('model_path', model_path)
        path = self.get_parameter('model_path').value
        self.declare_parameter('confidence', DEFAULT_CONFIDENCE)
        self.confidence = self.get_parameter('confidence').value

        # Subscription and publishers in a Reentrant group
        group = ReentrantCallbackGroup()
        self.bbox_publisher = self.create_publisher(
            Quaternion, 'bbox', 100
        )
        self.image_publisher = self.create_publisher(
            Image, 'yolo/image_raw', 100
        )
        self.bridge = CvBridge()

        # Load HEF and configure Hailo PCIe device
        self.hef = HEF(path)
        self.get_logger().info(f'Loaded HEF model from {path}')
        devices = Device.scan()
        if not devices:
            raise RuntimeError("No Hailo device found over PCIe")
        self.vdev = VDevice(device_ids=devices)
        self.vdev.__enter__()

        cfg = ConfigureParams.create_from_hef(self.hef, interface=HailoStreamInterface.PCIe)
        self.net_group, = self.vdev.configure(self.hef, cfg)
        self.ng_params = self.net_group.create_params()

        # Build stream parameters
        self.in_info = self.hef.get_input_vstream_infos()[0]
        self.out_info = self.hef.get_output_vstream_infos()[0]
        self.in_params = InputVStreamParams.make_from_network_group(self.net_group, quantized=False, format_type=FormatType.FLOAT32)
        self.out_params = OutputVStreamParams.make_from_network_group(self.net_group, quantized=False, format_type=FormatType.FLOAT32)

        # Build Hailo inference pipeline and activate the network
        self.infer_pipe = InferVStreams(self.net_group, self.in_params, self.out_params, tf_nms_format=True)
        self.infer_pipe.__enter__()
        self.activate_ctx = self.net_group.activate(self.ng_params)
        self.activate_ctx.__enter__()
        self.get_logger().info('Hailo inference pipeline activated')

        # PiCamera2 setup
        self.picam2 = Picamera2()
        cam_cfg = self.picam2.create_video_configuration(
            main={"size": (2304, 1296), "format": "RGB888"},
            lores={"size": (640, 360), "format": "RGB888"},
            controls={"FrameRate": 30.0},
            # transform=Transform(vflip=True, hflip=True)
        )
        self.picam2.configure(cam_cfg)
        self.picam2.start()
        self.get_logger().info('YOLO Node Start Detecting...')

        # Timer for pulling frames (non-blocking)
        self.timer = self.create_timer(
            0.05, self.timer_callback, callback_group=group
        )

        # Supplementary folder for saving undetected images
        self.folder = "/home/pivi/sandbox/trashcan"
        os.makedirs(self.folder, exist_ok=True)
        self.img_count = 0

    def timer_callback(self):
        try:
            # Capture frame
            # frame = self.picam2.capture_array()
            frame = self.picam2.capture_array('lores')
            orig = frame.copy()

            # Preprocess
            resized = cv2.copyMakeBorder(frame, 140, 140, 0, 0, borderType=cv2.BORDER_CONSTANT, value=0)
            nhwc = resized.astype(np.float32)
            inp = {self.in_info.name: np.expand_dims(nhwc, 0)}   # (1,H,W,3)

            # Inference
            output = self.infer_pipe.infer(inp)
            key = next(iter(output))
            data = output[key][0] # size = (number of classes, 5, number of detected objects)

            # Parse detections
            conf_map = data[:, 4, :]
            cls_idxs, det_idxs = np.where(conf_map > self.confidence)

            if cls_idxs.size:
                # (y1, x1, y2, x2, conf)
                y1 = (data[cls_idxs, 0, det_idxs] * 640 - 140) / 360
                x1 = data[cls_idxs, 1, det_idxs]
                y2 = (data[cls_idxs, 2, det_idxs] * 640 - 140) / 360
                x2 = data[cls_idxs, 3, det_idxs]
                confs = conf_map[cls_idxs, det_idxs]

                boxes = np.stack([x1, y1, x2, y2], axis=1).tolist()
                scores = confs.tolist()
                classes = cls_idxs.tolist()

                # Pick top-1
                best = int(np.argmax(scores))
                best_box, best_score, best_class = [boxes[best]], [scores[best]], [classes[best]]
                # draw_bboxes(orig, best_box, best_score, best_class)
                # draw_bboxes(orig, boxes, scores, classes)
                # cv2.imwrite(f"/home/pivi/sandbox/trashcan/image{self.img_count}.jpg", orig)
                # self.img_count += 1

                # Publish bounding box as Quaternion (x1,y1,x2,y2)
                bbox = Quaternion(x=best_box[0][0], y=best_box[0][1], z=best_box[0][2], w=best_box[0][3])
            else:
                # No detection: publish NaNs
                bbox = Quaternion(x=float('nan'), y=float('nan'), z=float('nan'), w=float('nan'))
            self.bbox_publisher.publish(bbox)

        except Exception as e:
            self.get_logger().error(f'Error in timer_callback: {e}')

    def destroy_node(self):
        # Clean up Hailo contexts
        try:
            self.activate_ctx.__exit__(None, None, None)
            self.infer_pipe.__exit__(None, None, None)
            self.vdev.__exit__(None, None, None)
        except Exception:
            pass
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
        node.get_logger().info('YOLO Node Shutting Down...')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()