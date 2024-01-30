import fcntl
import socket
import struct

import rclpy
from rclpy.node import Node
from rov_msgs.msg import IPAddress


class IPPublisher(Node):

    def __init__(self) -> None:
        """Create IP Publisher node."""
        super().__init__('ip_publisher')
        self.publisher_ = self.create_publisher(IPAddress, 'ip_address', 10)
        timer_period = 0.5  # seconds
        self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self) -> None:
        """On timer publishes the ip address of the computer."""
        msg = IPAddress()
        try:
            msg.address = get_ip_address()
        except OSError:
            self.get_logger().error("No such device for reading ip address.")
        self.publisher_.publish(msg)


# https://stackoverflow.com/questions/24196932/how-can-i-get-the-ip-address-from-a-nic-network-interface-controller-in-python
def get_ip_address(ifname: str = 'eth0') -> str:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15].encode())
    )[20:24])


def main() -> None:
    rclpy.init()
    minimal_publisher = IPPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
