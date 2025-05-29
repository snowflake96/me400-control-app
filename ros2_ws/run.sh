# #!/bin/bash

# export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"

# sudo chmod 777 /dev/video0
# sudo chmod 777 /dev/i2c-1

# # Get the directory where this script is located
# SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[1]}" )" && pwd )"
# # Source ROS2 and workspace
# source /opt/ros/jazzy/setup.bash
# source "$SCRIPT_DIR/install/setup.bash"
# sudo systemctl enable pigpiod
# sudo systemctl start pigpiod
# ros2 launch cannon_package cannon_launch.py

#!/bin/bash

export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"

# Give permissions
# sudo chmod 777 /dev/video0
sudo chmod 777 /dev/i2c-1

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Source ROS2 and workspace
source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

# # Check pigpiod status
# if ! pgrep -x "pigpiod" > /dev/null; then
#     echo "[INFO] pigpiod not running, attempting to start..."
#     sudo systemctl reset-failed pigpiod
#     sudo systemctl start pigpiod

#     sleep 1  # Give time to start

#     if ! pgrep -x "pigpiod" > /dev/null; then
#         echo "[ERROR] Failed to start pigpiod. Please check daemon setup or hardware."
#         exit 1
#     fi
# else
#     echo "[INFO] pigpiod is already running."
# fi

# Launch ROS2
ros2 launch cannon_package cannon_launch.py
