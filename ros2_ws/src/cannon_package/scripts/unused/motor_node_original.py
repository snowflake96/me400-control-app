#!/usr/bin/env python3
'''
This ROS2 node subscribes to the 'motor_command' topic and sets PWM
on four GPIO pins using lgpio (software-timed via DMA/timers) on Raspberry Pi 5.
'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import lgpio

# PWM settings
PWM_FREQUENCY = 1000  # Hz
# List your BCM GPIO pins here for channels 0–3:
PWM_PINS = [17, 18, 27, 22]

class MotorCommandSubscriber(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # 1) Open the GPIO chip (usually chip 0 on a Pi)
        try:
            self.h = lgpio.gpiochip_open(0)
        except Exception as e:
            self.get_logger().error(f"Failed to open GPIO chip: {e}")
            raise

        # 2) Initialize all PWM channels at 0% duty
        for pin in PWM_PINS:
            # freq=0 disables PWM, so we start by enabling at 0%
            lgpio.tx_pwm(self.h, pin, PWM_FREQUENCY, 0)

        # 3) ROS2 subscription
        self.subscription = self.create_subscription(
            Vector3,
            'motor_command',
            self.listener_callback,
            10
        )
        self.get_logger().info(f"🟢 MotorController initialized on pins {PWM_PINS}")

    def listener_callback(self, msg: Vector3):
        # Clamp inputs to [0.0, 1.0]
        duties = [
            max(0.0, min(1.0, msg.x)),
            max(0.0, min(1.0, msg.y)),
            max(0.0, min(1.0, msg.z))
        ]
        # Fourth channel = average of the three
        duties.append(sum(duties) / 3.0)

        self.get_logger().info(
            f"Received motor command → duties: {[f'{d:.2f}' for d in duties]}"
        )

        # Update each PWM channel
        for pin, duty in zip(PWM_PINS, duties):
            # lgpio.tx_pwm(handle, gpio, frequency_Hz, duty_cycle_pct)
            lgpio.tx_pwm(self.h, pin, PWM_FREQUENCY, duty * 100.0)

    def destroy_node(self):
        # Stop all PWMs
        for pin in PWM_PINS:
            lgpio.tx_pwm(self.h, pin, 0, 0)
        # Close the GPIO chip handle
        lgpio.gpiochip_close(self.h)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
