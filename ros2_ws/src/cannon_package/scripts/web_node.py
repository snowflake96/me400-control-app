#!/usr/bin/env python3

import os
import rclpy
from cannon_package.pid_webserver import PIDPublisher
import threading
from flask import Flask, request, render_template, jsonify
from ament_index_python.packages import get_package_share_directory

# Get the package share directory where your templates and static are installed
share_dir = get_package_share_directory('cannon_package')
templates_path = os.path.join(share_dir, 'templates')
static_path = os.path.join(share_dir, 'static')

# Create Flask app with explicit template and static folder paths
app = Flask(__name__, template_folder=templates_path, static_folder=static_path)
pid_node = None  # This will hold the ROS node

@app.route('/')
def index():
    # Pass both controllers' PID values to the template
    return render_template('index.html', pid=pid_node.pid_values, params=pid_node.params_values)

@app.route('/set_pid', methods=['POST'])
def set_pid():
    data = request.json
    # Determine which controller to update; default to 'position' if not provided.
    controller = data.get("controller", "position").lower()
    if controller not in ["position", "velocity"]:
        controller = "position"
    
    pid_node.pid_values[controller]["kp"] = float(data.get("kp", pid_node.pid_values[controller]["kp"]))
    pid_node.pid_values[controller]["ki"] = float(data.get("ki", pid_node.pid_values[controller]["ki"]))
    pid_node.pid_values[controller]["kd"] = float(data.get("kd", pid_node.pid_values[controller]["kd"]))

    # Immediately publish the new PID values for the specified controller
    pid_node.publish_pid(controller)
    
    # print(f"Updated PID values for {controller}: {pid_node.pid_values[controller]}")
    return jsonify(success=True, pid=pid_node.pid_values[controller], controller=controller)

@app.route('/set_params', methods=['POST'])
def set_params():
    data = request.json
    pid_node.params_values["cutoff_freq"] = float(data.get("cutoff_freq", pid_node.params_values["cutoff_freq"]))
    pid_node.params_values["sampling_time"] = float(data.get("sampling_time", pid_node.params_values["sampling_time"]))
    pid_node.params_values["integral_limit"] = float(data.get("integral_limit", pid_node.params_values["integral_limit"]))

    # Immediately publish the new parameter values
    pid_node.publish_params()
    
    # print(f"Updated Params values: {pid_node.params_values}")
    return jsonify(success=True, params=pid_node.params_values)

def run_flask():
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)
    except Exception as e:
        print("Flask failed to start:", e)

def main():
    global pid_node
    rclpy.init()
    pid_node = PIDPublisher()
    
    # Start Flask in a separate thread
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.daemon = True
    flask_thread.start()

    # Keep the ROS 2 node running
    rclpy.spin(pid_node)

    # Shutdown
    pid_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
