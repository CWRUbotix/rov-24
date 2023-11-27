import rclpy
from rclpy.node import Node

from rov_msgs.msg import Manip

from smbus2 import SMBus

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

        with SMBus(TCA9555_ADDRESS) as bus:
            pass

    def manip_callback(self, msg: Manip) -> None:
        pass


def main(args=None) -> None:
    rclpy.init(args=args)

    minimal_subscriber = Manipulator()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
