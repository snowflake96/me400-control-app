#!/usr/bin/env python3
import os
import signal
import rclpy
from ament_index_python.packages import get_package_share_directory

from cannon_package.yolo import YoloPublisher

def main():
    rclpy.init()

    share_dir = get_package_share_directory('cannon_package')
    # model_path = os.path.join(share_dir, 'model', 'final_bell_initial_openvino_model')
    model_path = os.path.join(share_dir, 'model', 'yolov8n.hef')

    node = YoloPublisher(model_path)

    def _shutdown(signum, frame):
        node.get_logger().info('Shutdown signal received, shutting down...')
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()


# import os
# import threading
# import signal
# import subprocess
# import sys

# import rclpy
# from flask import Flask, Response
# from ament_index_python.packages import get_package_share_directory

# from cannon_package.yolo import get_frame, YoloPublisher

# def cleanup(signum, frame):
#     print("[Cleanup] Caught Ctrl+C. Killing gunicorn...")
#     subprocess.run(['pkill', '-f', 'gunicorn'], check=False)
#     sys.exit(0)
# # Register the signal handler
# signal.signal(signal.SIGINT, cleanup)
# signal.signal(signal.SIGTERM, cleanup)
# #
# # 1) Start your ROS2 node + detection thread immediately on import
# #
# rclpy.init()
# share_dir = get_package_share_directory('cannon_package')
# model_path = os.path.join(share_dir, 'model', 'final_bell_initial_openvino_model')

# node = YoloPublisher(model_path)
# threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

# def _shutdown(signum, frame):
#     node.get_logger().info('Shutdown signal received')
#     rclpy.shutdown()
# signal.signal(signal.SIGINT,  _shutdown)
# signal.signal(signal.SIGTERM, _shutdown)

# #
# # 2) Create the Flask WSGI app at module scope
# #
# app = Flask(__name__)

# @app.route('/health')
# def health():
#     return 'OK', 200

# @app.route('/')
# def stream():
#     def generate():
#         while True:
#             jpg = get_frame()
#             if jpg is None:
#                 continue
#             yield (
#                 b'--frame\r\n'
#                 b'Content-Type: image/jpeg\r\n\r\n' +
#                 jpg +
#                 b'\r\n'
#             )
#     return Response(
#         generate(),
#         mimetype='multipart/x-mixed-replace; boundary=frame'
#     )


# VERSION 2

# #!/usr/bin/env python3
# # File: app.py (revised Flask server)
# # This simple Flask app streams MJPEG frames produced by our YOLO publisher thread.
# # For production, use Gunicorn+Gevent:
# #   pip install gunicorn gevent
# #   gunicorn -w 1 -k gevent app:app

# import os
# import threading
# import time
# import signal

# import rclpy
# from flask import Flask, Response
# from ament_index_python.packages import get_package_share_directory

# from cannon_package.yolo import get_frame, YoloPublisher

# def create_app():
#     app = Flask(__name__)

#     @app.route('/health')
#     def health():
#         return 'OK', 200

#     @app.route('/')
#     def stream():
#         def generate():
#             while True:
#                 jpg = get_frame()
#                 if jpg is None:
#                     time.sleep(0.01)
#                     continue
#                 yield (
#                     b'--frame\r\n'
#                     b'Content-Type: image/jpeg\r\n\r\n' +
#                     jpg +
#                     b'\r\n'
#                 )
#         return Response(
#             generate(),
#             mimetype='multipart/x-mixed-replace; boundary=frame'
#         )

#     return app

# def main():
#     rclpy.init()

#     share_dir = get_package_share_directory('cannon_package')
#     model_path = os.path.join(share_dir, 'model', 'final_bell_initial_openvino_model')

#     # start YOLO node in background
#     node = YoloPublisher(model_path)
#     threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

#     host = node.declare_parameter('host', '0.0.0.0').value
#     port = node.declare_parameter('port', 3000).value

#     def _shutdown(signum, frame):
#         node.get_logger().info('Shutdown signal received')
#         rclpy.shutdown()

#     signal.signal(signal.SIGINT, _shutdown)
#     signal.signal(signal.SIGTERM, _shutdown)

#     # run Flask's built-in server
#     app = create_app()
#     app.run(host=host, port=port, threaded=True)

#     # cleanup
#     node.destroy_node()

# if __name__ == '__main__':
#     main()





