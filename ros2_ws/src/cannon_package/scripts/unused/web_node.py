#!/usr/bin/env python3
"""
Flask + ROS 2 bridge for tuning *pitch* and *yaw* PID loops.

Replaces the previous “position / velocity” controller pair.
"""

import os
import threading
from flask import Flask, request, render_template, jsonify
import rclpy
from ament_index_python.packages import get_package_share_directory
from cannon_package.pid_webserver import PIDPublisher

# ──────────────────────────────────────────────────────────────────────────────
#  Flask setup
# ──────────────────────────────────────────────────────────────────────────────
share_dir = get_package_share_directory("cannon_package")
templates_path = os.path.join(share_dir, "templates")
static_path = os.path.join(share_dir, "static")

app = Flask(__name__, template_folder=templates_path, static_folder=static_path)
pid_node = None        # will be initialised in main()


# ──────────────────────────────────────────────────────────────────────────────
#  Routes
# ──────────────────────────────────────────────────────────────────────────────
@app.route("/")
def index():
    # Pass current PID + filter parameters to the template
    return render_template(
        "index.html",
        pid=pid_node.pid_values,            # now holds “pitch” and “yaw”
        params=pid_node.params_values,
    )


@app.route("/set_pid", methods=["POST"])
def set_pid():
    data = request.json

    # Which loop are we updating?
    controller = data.get("controller", "pitch").lower()
    if controller not in {"pitch", "yaw"}:
        controller = "pitch"

    # Update gains
    gains = pid_node.pid_values[controller]
    gains["kp"] = float(data.get("kp", gains["kp"]))
    gains["ki"] = float(data.get("ki", gains["ki"]))
    gains["kd"] = float(data.get("kd", gains["kd"]))

    # Publish immediately
    pid_node.publish_pid(controller)

    return jsonify(success=True, pid=gains, controller=controller)


@app.route("/set_params", methods=["POST"])
def set_params():
    data = request.json
    params = pid_node.params_values
    params["cutoff_freq"]   = float(data.get("cutoff_freq",   params["cutoff_freq"]))
    params["sampling_time"] = float(data.get("sampling_time", params["sampling_time"]))
    params["integral_limit"] = float(
        data.get("integral_limit", params["integral_limit"])
    )

    pid_node.publish_params()
    return jsonify(success=True, params=params)


# ──────────────────────────────────────────────────────────────────────────────
#  Helper
# ──────────────────────────────────────────────────────────────────────────────
def run_flask():
    try:
        app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)
    except Exception as exc:
        print("Flask failed to start:", exc)


# ──────────────────────────────────────────────────────────────────────────────
#  Main
# ──────────────────────────────────────────────────────────────────────────────
def main():
    global pid_node
    rclpy.init()
    pid_node = PIDPublisher()   # make sure this class initialises “pitch” & “yaw”

    # Run Flask in its own thread
    threading.Thread(target=run_flask, daemon=True).start()

    # Spin ROS 2 node
    rclpy.spin(pid_node)

    # Cleanup
    pid_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
