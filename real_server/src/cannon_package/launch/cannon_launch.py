#!/usr/bin/env python3
"""
Launch file: cannon_launch.py
This will launch:
  - Your C++ main node
  - The web_node.py (Flask tuning GUI)
  - The Gunicorn+Gevent server for your yolo_node.py
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    pkg_share = get_package_share_directory('cannon_package')
    return LaunchDescription([
        # Launch the C++ main node
        Node(
            package='cannon_package',
            executable='main',
            output='screen',
        ),
        # # Launch the yolo node
        # Node(
        #     package='cannon_package',
        #     executable='yolo_node.py',
        #     name='cannon_yolo',
        #     output='screen',
        # ),

        # Node(
        #     package='cannon_package',
        #     executable='limit_switch_node.py',
        #     name='cannon_switch',
        #     output='screen',
        # )

        # Launch the Python PID tuning web node (Flask dev server)
        # Node(
        #     package='cannon_package',
        #     executable='web_node.py',
        #     name='cannon_web',
        #     output='screen',
        # ),

        # Launch the YOLO MJPEG stream under Gunicorn+Gevent
        # ExecuteProcess(
        #     cmd=[
        #         'gunicorn',
        #         '-w', '1',                  # one worker process
        #         '-k', 'gevent',             # gevent worker for async I/O
        #         'cannon_package.yolo_node:app',  # module:app
        #         '--bind', '0.0.0.0:3000',    # listen on port 3000
        #     ],
        #     cwd=os.path.join(get_package_prefix('cannon_package'), 'lib', 'cannon_package'),
        #     output='screen',
        #     log_cmd=True,
        # ),
        # Node(
        #     package='cannon_package',
        #     executable='mouse.py',
        #     name='cannon_mouse',
        #     output='screen',
        # ),
    ])
