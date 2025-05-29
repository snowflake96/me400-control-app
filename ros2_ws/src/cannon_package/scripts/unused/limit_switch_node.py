import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from gpiozero import Button, Device
from gpiozero.pins.lgpio import LGPIOFactory

class LimitSwitchNode(Node):
    PINS = [17, 18, 27, 22]  # BCM pin numbers for switches 0–3

    def __init__(self):
        super().__init__('limit_switch_node')
        Device.pin_factory = LGPIOFactory(chip=0)

        # Publishers now match your C++ subscriptions
        self.press_pub   = self.create_publisher(Int32, 'limit_switch_pressed', 10)
        self.release_pub = self.create_publisher(Int32, 'limit_switch_released', 10)

        # Create one Button per switch ID and wire callbacks
        self.switches = []
        for idx, pin in enumerate(self.PINS):
            btn = Button(pin, pull_up=True, bounce_time=0.05)
            btn.when_pressed  = lambda idx=idx: self._on_event(idx, pressed=True)
            btn.when_released = lambda idx=idx: self._on_event(idx, pressed=False)
            self.switches.append(btn)
            self.get_logger().info(f'Initialized switch {idx} on GPIO {pin}')

    def _on_event(self, switch_id: int, pressed: bool):
        msg = Int32(data=switch_id)
        if pressed:
            self.press_pub.publish(msg)
            self.get_logger().info(f'🔴 Switch {switch_id} PRESSED')
        else:
            self.release_pub.publish(msg)
            self.get_logger().info(f'🟢 Switch {switch_id} RELEASED')

def main(args=None):
    rclpy.init(args=args)
    node = LimitSwitchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for btn in node.switches:
            btn.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
