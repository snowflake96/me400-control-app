#!/usr/bin/env python3

'''
This node received motor commands and output pwm on gpio pins
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

class MotorCommandReceiver(Node):
    def __init__(self):
        super().__init__('motor_command_receiver')
        self.subscription = self.create_subscription(
            Vector3,
            'motor_commands',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        motor_1 = msg.x
        motor_2 = msg.y
        motor_3 = msg.z
        self.get_logger().info(f'Received Motor Commands - M1: {motor_1}, M2: {motor_2}, M3: {motor_3}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
