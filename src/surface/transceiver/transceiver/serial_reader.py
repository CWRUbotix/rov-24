import time

from serial import Serial
from serial.serialutil import SerialException
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

        first_attempt = True
        self.ser: Serial | None = None

        while not self.ser:
            try:
                self.ser = Serial('/dev/ttyTransceiver', 115200)
            except SerialException:
                if first_attempt:
                    self.get_logger().warning('Transceiver not plugged in.')
                    first_attempt = False
                time.sleep(1)

        self.create_timer(timer_period, self.timer_callback)

    @property
    def serial(self) -> Serial:
        return self._serial
    
    @property.setter
    def serial(self, Serial)

    def timer_callback(self) -> None:
        """Publish a message from the transceiver."""
        msg = FloatCommand()
        msg.command = self.ser.readline().decode()
        self.publisher.publish(msg)

    def control_callback(self, msg: FloatCommand) -> None:
        """
        Log a binary string of the command that was sent.

        Parameters
        ----------
        msg : FloatCommand
            the command that was sent via serial monitor

        """
        msg_encode: bytes = msg.command.encode()
        self.ser.write(msg_encode)
        # !r is to print a binary string
        self.get_logger().info(f'Command sent via serial monitor: {msg_encode!r}')


def main() -> None:
    """Run the serial reader node."""
    rclpy.init()

    serial_reader = SerialReader()

    rclpy.spin(serial_reader)


if __name__ == '__main__':
    main()
