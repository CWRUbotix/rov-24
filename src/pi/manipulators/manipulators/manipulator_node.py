import time

import rclpy
from rclpy.node import Node
from smbus2 import SMBus

from rov_msgs.msg import Manip

TCA9555_ADDRESS = 0x20


class Manipulator(Node):

    def __init__(self) -> None:
        super().__init__('manipulator',
                         parameter_overrides=[])

        self.subscription = self.create_subscription(
            Manip,
            'topic',
            self.manip_callback,
            10)

        with SMBus(0) as self.bus:
            while True:

                self.bus.write_byte(TCA9555_ADDRESS, 0b00000000)
                time.sleep(1000)
                self.bus.write_byte(TCA9555_ADDRESS, 0b11111111)

    def manip_callback(self, msg: Manip) -> None:
        # self.bus.
        # self.bus.write_byte()
        pass


def main() -> None:
    rclpy.init()

    minimal_subscriber = Manipulator()

    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
