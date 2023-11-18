import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import lgpio
import time

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('gpio_reader', parameter_overrides=[])
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        detect1, detect2, detect3 = 2,4,6
        h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_input(h,detect1)
        lgpio.gpio_claim_input(h,detect2)
        lgpio.gpio_claim_input(h,detect3)
        data1 = lgpio.gpio_read(h,detect1)
        data2 = lgpio.gpio_read(h,detect2)
        data3 = lgpio.gpio_read(h,detect3)

        msg = String()
        # msg.data = 'Hello World: %d' % self.i
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()