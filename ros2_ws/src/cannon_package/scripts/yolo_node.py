#!/usr/bin/env python3
import os
import threading
import time
import signal

import rclpy
from flask import Flask, Response
from waitress import create_server
from ament_index_python.packages import get_package_share_directory
import cv2

from cannon_package.yolo import YoloPublisher, get_frame

# Global handle for the Waitress server
flask_server = None

def create_app():
    app = Flask(__name__)

    @app.route('/health')
    def health():
        return 'OK', 200

    @app.route('/')
    def stream():
        def generate():
            while True:
                frame = get_frame()
                if frame is None:
                    time.sleep(0.1)
                    continue
                ret, buffer = cv2.imencode('.jpg', frame)
                if not ret:
                    continue
                yield (
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' +
                    buffer.tobytes() +
                    b'\r\n'
                )
        return Response(
            generate(),
            mimetype='multipart/x-mixed-replace; boundary=frame'
        )

    return app

def run_flask(host: str, port: int):
    global flask_server
    app = create_app()
    flask_server = create_server(app, host=host, port=port)
    print(f"* Running on http://{host}:{port}/", flush=True)
    flask_server.run()


def main():
    rclpy.init()

    share_dir = get_package_share_directory('cannon_package')
    model_path = os.path.join(share_dir, 'model', 'picam_bell_openvino_model')

    node = YoloPublisher(model_path)
    node.declare_parameter('host', '0.0.0.0')
    node.declare_parameter('port', 3000)
    host = node.get_parameter('host').value
    port = node.get_parameter('port').value

    def _shutdown(signum, frame):
        node.get_logger().info('Shutdown signal received, shutting down...')
        if flask_server:
            flask_server.close()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    flask_thread = threading.Thread(
        target=run_flask,
        args=(host, port),
        daemon=True
    )
    flask_thread.start()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        flask_thread.join(timeout=1)

if __name__ == '__main__':
    main()