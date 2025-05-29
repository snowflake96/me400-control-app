import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from adafruit_servokit import ServoKit

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        
        # Initialize the PCA9685 servo driver on I2C bus 1
        self.kit = ServoKit(channels=16)  # Default is using I2C bus 1

        # Set frequency for servo updates
        self.kit.frequency = 50  # Set the frequency to 50Hz

        self.get_logger().info('PCA9685 initialized at 50Hz')

        # Create a subscription to the "motor_command" topic
        self.subscription = self.create_subscription(
            Vector3,
            'motor_command',
            self.on_command,
            10  # Queue size
        )
        self.get_logger().info("Subscribed to 'motor_command' topic")

    def on_command(self, msg: Vector3):
        # Map x→channel0, y→channel1, z→channel2
        self.kit.servo[0].angle = msg.x
        self.kit.servo[1].angle = msg.y
        self.kit.servo[2].angle = msg.z
        self.get_logger().info(f"Command received: x={msg.x}, y={msg.y}, z={msg.z}")

def main(args=None):
    rclpy.init(args=args)
    
    node = ServoNode()

    # Spin the node to keep receiving messages
    rclpy.spin(node)

    # Shutdown the node gracefully
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
