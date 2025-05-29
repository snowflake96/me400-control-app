#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

class PIDPublisher(Node):
    def __init__(self):
        super().__init__('pid_publisher')
        self.get_logger().info("PID Publisher node initialized")
        # Create publishers for two separate PID controllers: position and velocity
        self.publisher_position_ = self.create_publisher(Vector3, 'pid_gains_position', 10)
        self.publisher_velocity_ = self.create_publisher(Vector3, 'pid_gains_velocity', 10)
        # Publisher for parameter values (shared)
        self.params_publisher_ = self.create_publisher(Vector3, 'pid_params_topic', 10)
        
        # Default PID values for two controllers
        self.pid_values = {
            "position": {"kp": 1.0, "ki": 0.0, "kd": 0.0},
            "velocity": {"kp": 1.0, "ki": 0.0, "kd": 0.0}
        }
        # Default parameter values
        self.params_values = {"cutoff_freq": 100.0, "sampling_time": 0.01, "integral_limit": 10.0}

    def publish_pid(self, controller):
        msg = Vector3()
        gains = self.pid_values[controller]
        msg.x = gains["kp"]
        msg.y = gains["ki"]
        msg.z = gains["kd"]
        if controller == "position":
            self.publisher_position_.publish(msg)
        elif controller == "velocity":
            self.publisher_velocity_.publish(msg)
        self.get_logger().info(f'Published PID for {controller}: [{msg.x}, {msg.y}, {msg.z}]')

    def publish_params(self):
        msg = Vector3()
        msg.x = self.params_values["cutoff_freq"]
        msg.y = self.params_values["sampling_time"]
        msg.z = self.params_values["integral_limit"]
        self.params_publisher_.publish(msg)
        self.get_logger().info(f'Published Params: [{msg.x}, {msg.y}, {msg.z}]')

def main(args=None):
    rclpy.init(args=args)
    pid_publisher = PIDPublisher()
    rclpy.spin(pid_publisher)
    pid_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
