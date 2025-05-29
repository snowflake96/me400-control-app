#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import pygame

class MousePositionPublisher(Node):
    def __init__(self):
        super().__init__('mouse_position_publisher')
        # create publisher
        self.pub = self.create_publisher(Vector3, 'mouse_position', 10)

        # Pygame setup
        self.window_size = 600
        self.fps = 60
        pygame.init()
        self.screen = pygame.display.set_mode(
            (self.window_size, self.window_size)
        )
        pygame.display.set_caption('Mouse Position Publisher')

        # timer to drive read+publish
        self.timer = self.create_timer(1.0 / self.fps, self.publish_mouse_pos)

        self.get_logger().info('🟢 MOUSE!')

    def publish_mouse_pos(self):
        # pump pygame events so window stays responsive
        pygame.event.pump()

        # read pixel coords
        mx, my = pygame.mouse.get_pos()

        # normalize to [-1,1]; invert Y so up is +1
        nx = (mx / (self.window_size - 1)) * 2.0 - 1.0
        ny = 1.0 - (my / (self.window_size - 1)) * 2.0

        self.get_logger().info(f"🎯 Mouse normalized pos → x: {nx:.3f}, y: {ny:.3f}")

        # pack into Vector3 (z ignored = 0)
        msg = Vector3()
        msg.x = nx
        msg.y = ny
        msg.z = 0.0

        self.pub.publish(msg)

        # optional: clear screen each frame
        self.screen.fill((0, 0, 0))
        pygame.display.flip()

def main(args=None):
    rclpy.init(args=args)
    node = MousePositionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        pygame.quit()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
