#!/usr/bin/env python3
'''
This ROS2 node subscribes to the 'motor_command' topic and sets PWM
on four GPIO pins using lgpio (software-timed via DMA/timers) on Raspberry Pi 5.
'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import threading
import lgpio
import time

# PWM settings
PWM_FREQUENCY = 1000  # Hz
# List your BCM GPIO pins here for channels 0–3:
# PWM_PINS = [17, 18, 27, 22]
PWM_PINS = [22, 27]

class MotorCommandSubscriber(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # 1) Open the GPIO chip (usually chip 0 on a Pi)
        try:
            self.h = lgpio.gpiochip_open(4)
        except Exception as e:
            self.get_logger().error(f"Failed to open GPIO chip: {e}")
            raise

        # 2) Initialize all PWM channels at 0% duty
        for pin in PWM_PINS:
            # freq=0 disables PWM, so we start by enabling at 0%
            lgpio.gpio_claim_output(self.h, pin, 0)
            lgpio.tx_pwm(self.h, pin, PWM_FREQUENCY, 0)

        self.direction = 1
        self.lock = threading.Lock()

        # 3) ROS2 subscription
        self.subscription = self.create_subscription(
            Vector3,
            'motor_command',
            self.direction_callback,
            10
        )
        self.get_logger().info(f"🟢 MotorController initialized on pins {PWM_PINS}")

        self._stop_thread = False
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()
    
    def direction_callback(self, msg):
        self.direction = msg.z
        self.get_logger().info(f"Motor node get {self.direction}")
        return 0

        # 710 ~ 500 for CCW / 780, 990 for CW
    def calculate_pwm_duty(self, min_duty, max_duty, percent):
        duty = min_duty + (percent / 100) * (max_duty - min_duty)
        duty = duty / 10
        return duty
    
    def control_loop(self):
        pin1 = 22
        pin2 = 27
        percent = 10
        while rclpy.ok() and not self._stop_thread:
            with self.lock:
                dir = self.direction
            if dir == 0:
                value1 = self.calculate_pwm_duty(550, 710, 100-percent)
                value2 = self.calculate_pwm_duty(780, 990, percent)
                self.get_logger().info(f"motor value1 = {value1}")
                self.get_logger().info(f"motor value2 = {value2}")
                lgpio.tx_pwm(self.h, pin1, PWM_FREQUENCY, value1)
                lgpio.tx_pwm(self.h, pin2, PWM_FREQUENCY, value2)
            else:
                lgpio.tx_pwm(self.h, pin1, PWM_FREQUENCY, 0)
                lgpio.tx_pwm(self.h, pin2, PWM_FREQUENCY, 0)


    def destroy_node(self):
        # Stop all PWMs
        self._stop_thread = True
        self.control_thread.join()
        for pin in PWM_PINS:
            lgpio.tx_pwm(self.h, pin, 0, 0)
        # Close the GPIO chip handle
        lgpio.gpiochip_close(self.h)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandSubscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        # rclpy.spin(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
