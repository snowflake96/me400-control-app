from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the C++ node (executable "main")
        Node(
            package='cannon_package',
            executable='main',
            name='cannon_main',
            output='screen',
        ),
        # Launch the Python node (installed as "web_node.py")
        Node(
            package='cannon_package',
            executable='web_node.py',
            name='cannon_web',
            output='screen',
        ),
        # Launch the YOLO node
        Node(
            package='cannon_package',
            executable='yolo_node.py',
            name='cannon_yolo',
            output='screen',
        ),
        # # Launch v4l2_camera node
        # Node(
        #     package='v4l2_camera',
        #     executable='v4l2_camera_node',
        #     name='camera',
        #     output='screen',
        #     parameters=[{
        #         'image_size': [640, 480],
        #         'camera_frame_id': 'camera_frame'
        #     }]
        # )
    ])
