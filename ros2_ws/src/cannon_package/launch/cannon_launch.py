#!/usr/bin/env python3
"""
Launch file: cannon_launch.py
This will launch:
  - Your C++ main node (which includes master, motor, and IMU nodes)
  - The yolo node for object detection
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the path to the source directory by going up from the install directory
    pkg_share = get_package_share_directory('cannon_package')
    workspace_dir = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
    params = os.path.join(workspace_dir, 'src', 'cannon_package', 'config', 'params.yaml')
    
    print(f"Loading parameters from: {params}")  # Debug print

    return LaunchDescription([
        # Launch the C++ main (which includes master, motor, and IMU nodes)
        Node(
            package='cannon_package',
            executable='main',
            output='screen',
            parameters=[params],  # Parameters for both master and motor nodes
        ),

        # Launch the yolo node
        Node(
            package='cannon_package',
            executable='yolo_node.py',
            name='cannon_yolo',
            output='screen',
        ),

        # Commented out nodes can be uncommented and configured as needed
        # Node(
        #     package='cannon_package',
        #     executable='mouse.py',
        #     name='cannon_mouse',
        #     output='screen',
        # ),

        # Node(
        #     package='cannon_package',
        #     executable='limit_switch_node',
        #     name='cannon_switch',
        #     output='screen',
        # )
    ])
