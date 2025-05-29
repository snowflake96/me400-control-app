#!/bin/bash

export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"

sudo modprobe i2c_dev
sudo modprobe i2c_bcm2835

# Kill any process using port 3000 to prevent address already in use errors
fuser -k 3000/tcp 2>/dev/null || true
pkill -f gunicorn || true
# ——————————————————————————————
# 2) Give permissions to I2C
# ——————————————————————————————
# sudo chmod 666 /dev/i2c-1

# ——————————————————————————————
# 3) Source ROS2 and your workspace
# ——————————————————————————————
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

# 1) Define a trap for SIGINT in the script.  
#    This handler will run *after* ros2 launch dies, because catch dispositions
#    on execve are reset to default in the child—so ros2 still sees SIGINT.
trap 'printf "\n[INFO] Caught Ctrl-C—ros2 launch exited, stopping motors...\n"' SIGINT

# ——————————————————————————————
# 4) Launch your ROS2 system
# ——————————————————————————————
echo "[INFO] Launching cannon_package..."
ros2 launch cannon_package cannon_launch.py

# Run script to stop all motors
./stop