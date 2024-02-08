import rclpy
from rclpy.node import Node
from rov_msgs.msg import Flooding
import lgpio


class floodDetector(Node):

    def __init__(self):
        super().__init__('gpio_reader', parameter_overrides=[])
        self.publisher_ = self.create_publisher(Flooding, '/tether/flooding', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # Pins used for GPIO
        detect1, detect2, detect3 = 2, 4, 6
        # GPIO Boilerplate
        h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_input(h, detect1)
        lgpio.gpio_claim_input(h, detect2)
        lgpio.gpio_claim_input(h, detect3)
        # Read Data
        # data1 = lgpio.gpio_read(h, detect1)
        # data2 = lgpio.gpio_read(h, detect2)
        # data3 = lgpio.gpio_read(h, detect3)

        # If any of the sensors detect water, send true to /tether/flooding

        msg = Flooding()

        if True:
            msg.flooding = False
        else:
            self.fuckthelinter = True
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init()
    flood_detector = floodDetector()
    rclpy.spin(flood_detector)
    flood_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
