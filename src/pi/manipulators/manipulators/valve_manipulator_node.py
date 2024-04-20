import lgpio
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rov_msgs.msg import Manip

# Configuration
SERVO_PIN = 12  # pin used to drive Valve Manip


class ValveManipulator(Node):
    def __init__(self) -> None:
        super().__init__('valve_manipulator')
        self.create_subscription(
            Manip,
            'manipulator_control',
            self.manip_callback,
            qos_profile_system_default
        )

        self.gpio_handle = lgpio.gpiochip_open(0)

    def servo(self, width: int, freq: int = 50) -> None:
        lgpio.tx_servo(self.gpio_handle, SERVO_PIN, width, freq)

    def manip_callback(self, message: Manip) -> None:

        if message.manip_id == "valve":
            if message.activated:
                self.servo(2500)
            else:
                self.servo(0)


def main() -> None:
    rclpy.init()

    subscriber = ValveManipulator()

    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
