#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

class PIDPublisher(Node):
    def __init__(self):
        super().__init__('pid_publisher')
        self.get_logger().info("PID Publisher node initialized")

        # Create publishers for two separate PID controllers: pitch and yaw
        self.publisher_pitch_ = self.create_publisher(Vector3, 'pid_gains_pitch', 10)
        self.publisher_yaw_   = self.create_publisher(Vector3, 'pid_gains_yaw',   10)

        # Publisher for shared parameter values
        self.params_publisher_ = self.create_publisher(Vector3, 'pid_params_topic', 10)

        # Default PID values for pitch and yaw
        self.pid_values = {
            "pitch": {"kp": 1.0, "ki": 0.0, "kd": 0.0},
            "yaw":   {"kp": 1.0, "ki": 0.0, "kd": 0.0}
        }

        # Default shared parameter values
        self.params_values = {
            "cutoff_freq":   100.0,
            "sampling_time": 0.01,
            "integral_limit": 10.0
        }

    def publish_pid(self, controller: str):
        msg = Vector3()
        gains = self.pid_values[controller]
        msg.x = gains["kp"]
        msg.y = gains["ki"]
        msg.z = gains["kd"]

        if controller == "pitch":
            self.publisher_pitch_.publish(msg)
        elif controller == "yaw":
            self.publisher_yaw_.publish(msg)
        else:
            self.get_logger().warn(f"Unknown controller '{controller}'")

        self.get_logger().info(
            f"Published PID for {controller}: [kp={msg.x}, ki={msg.y}, kd={msg.z}]"
        )

    def publish_params(self):
        msg = Vector3()
        msg.x = self.params_values["cutoff_freq"]
        msg.y = self.params_values["sampling_time"]
        msg.z = self.params_values["integral_limit"]
        self.params_publisher_.publish(msg)
        self.get_logger().info(
            f"Published Params: [cutoff_freq={msg.x}, sampling_time={msg.y}, integral_limit={msg.z}]"
        )

def main(args=None):
    rclpy.init(args=args)
    pid_publisher = PIDPublisher()
    rclpy.spin(pid_publisher)
    pid_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
