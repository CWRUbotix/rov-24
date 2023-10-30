import rclpy
from rclpy.node import Node

from rov_msgs.msg import Manip

import lgpio


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber',
                         parameter_overrides=[])
        self.subscription = self.create_subscription(
            Manip,
            'topic',
            self.listener_callback,
            10)
        i2c = lgpio.i2c_open()

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
