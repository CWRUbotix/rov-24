import serial
import rclpy
from rclpy.node import Node
from rov_msgs.msg import FloatCommand


class SerialReader(Node):

    def __init__(self) -> None:
        super().__init__('serial_reader',
                         parameter_overrides=[])
        self.publisher = self.create_publisher(FloatCommand, 'transceiver_data', 10)
        self.listener = self.create_subscription(
            FloatCommand,
            'transceiver_control',
            self.control_callback,
            10)
        timer_period = .5
        self.ser = serial.Serial('/dev/ttyTransceiver', 115200)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self) -> None:
        msg = FloatCommand()
        msg.command = self.ser.readline().decode()
        self.publisher.publish(msg)

    def control_callback(self, msg: FloatCommand) -> None:
        msg_encode: bytes = msg.command.encode()
        self.ser.write(msg_encode)
        # !r is to print a binary string
        self.get_logger().info(f'Command sent via serial monitor: {msg_encode!r}')


def main() -> None:
    rclpy.init()

    serial_reader = SerialReader()

    rclpy.spin(serial_reader)


if __name__ == '__main__':
    main()
