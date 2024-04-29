import time
from math import floor
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from serial import Serial
from serial.serialutil import SerialException

from rov_msgs.msg import FloatData

MILLISECONDS_TO_SECONDS = 1/1000
MBAR_TO_METER_OF_HEAD = 0.010199773339984


class SerialReader(Node):

    def __init__(self) -> None:
        super().__init__('serial_reader')
        self.publisher = self.create_publisher(FloatData, 'transceiver_data', qos_profile_sensor_data)
        timer_period = .5

        self.first_attempt = True
        self.create_timer(timer_period, self.timer_callback)
        self._serial: Optional[Serial] = None

    @property
    def serial(self) -> Serial:
        while not self._serial:
            try:
                self._serial = Serial('/dev/ttyTransceiver', 115200)
            except SerialException:
                if self.first_attempt:
                    self.get_logger().warning('Transceiver not plugged in.')
                    self.first_attempt = False
                time.sleep(1)
        return self._serial

    def timer_callback(self) -> None:
        """Publish a message from the transceiver."""
        msg = FloatData()

        packet = self.serial.readline()

        # TODO handle readline returning empty byte

        msg.is_empty = False

        prefix = packet[0:FloatData.PREFIX_LENGTH]
        data = packet[FloatData.PREFIX_LENGTH:]

        # Better to round down and miss one set of data
        data_size = floor(len(data) / 2)

        if data_size > FloatData.DATA_MAX_LENGTH:
            self.get_logger().warning("Somehow data is longer than max data size")

        msg.team_number = int(prefix[0:1])
        msg.profile_number = int(prefix[1:2])

        # Byte 3 is no longer being used

        # Starts out as unsigned long
        msg.depth_data = [int(data) * MBAR_TO_METER_OF_HEAD for data in data[0:data_size]]
        # Starts out as float32
        msg.time_data = [float(data) *
                         MILLISECONDS_TO_SECONDS for data in data[data_size:2*data_size]]

        self.publisher.publish(msg)


def main() -> None:
    """Run the serial reader node."""
    rclpy.init()

    serial_reader = SerialReader()

    rclpy.spin(serial_reader)


if __name__ == '__main__':
    main()
