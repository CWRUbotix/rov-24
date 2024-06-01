import time
from threading import Thread
from queue import Queue
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from serial import Serial

from rov_msgs.msg import FloatCommand, FloatData, FloatSerial

MILLISECONDS_TO_SECONDS = 1/1000
SECONDS_TO_MINUTES = 1/60
MBAR_TO_METER_OF_HEAD = 0.010199773339984


AMBIENT_PRESSURE_DEFAULT = 1013.25  # in (mbar)

# AKA the length of the float in (m)
FLOAT_CONVERSION_FACTOR = 0.635


ROS_PACKET = "ROS:"
SECTION_SEPARATOR = ":"
DATA_SEPARATOR = ";"
COMMA_SEPARATOR = ","
HEADER_LENGTH = 3
PACKET_SECTIONS = 3


class SerialReader(Node):

    def __init__(self) -> None:
        super().__init__('serial_reader')
        self.data_publisher = self.create_publisher(FloatData, 'transceiver_data',
                                                    QoSPresetProfiles.SENSOR_DATA.value)

        self.create_subscription(FloatCommand, 'float_command', self.send_command,
                                 QoSPresetProfiles.DEFAULT.value)

        self.serial_publisher = self.create_publisher(FloatSerial, 'float_serial',
                                                      QoSPresetProfiles.SENSOR_DATA.value)

        self.serial = Serial("/dev/serial/by-id/usb-Adafruit_Feather_32u4-if00", 115200)

        self.surface_pressure = AMBIENT_PRESSURE_DEFAULT
        self.surface_pressures: Queue[float] = Queue(5)

    def start(self) -> None:
        Thread(target=self.read_serial, daemon=True,
               name="Serial Reader").start()

    def send_command(self, msg: FloatCommand) -> None:
        self.serial.write(msg.command.encode())

    def read_serial(self) -> None:
        buffer = b''
        while True:
            buffer += self.serial.read(self.serial.in_waiting)

            while b'\n' in buffer:
                packet, buffer = buffer.split(b'\n', 1)
                self.ros_publish(packet.decode())
            time.sleep(0.05)

    def ros_publish(self, packet: str) -> None:
        """Publish a message from the transceiver."""
        self.serial_publisher.publish(FloatSerial(serial=packet))

        if packet[:len(ROS_PACKET)] != ROS_PACKET:
            return

        try:
            if SerialReader.is_ros_single_message(packet):
                self._handle_ros_single(packet)
            else:
                msg = self._message_parser(packet)
                self.data_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error {e} caught dropping packet")

    @staticmethod
    def is_ros_single_message(packet: str) -> bool:
        ros_single = ROS_PACKET + "SINGLE"
        return packet[:len(ros_single)] != ros_single

    def _handle_ros_single(self, packet: str) -> None:
        if self.surface_pressures.full():
            return

        packet_sections = packet.split(SECTION_SEPARATOR)
        data = packet_sections[3]
        pressure = float(data.split(COMMA_SEPARATOR)[1])

        self.surface_pressures.put(pressure)

        q = self.surface_pressures.queue
        avg_pressure = sum(q) / len(q)
        self.surface_pressure = avg_pressure

    def _message_parser(self, packet: str) -> FloatData:
        msg = FloatData()

        packet_sections = packet.split(SECTION_SEPARATOR)

        if len(packet_sections) != PACKET_SECTIONS:
            raise ValueError(f"Packet expected {PACKET_SECTIONS} sections, "
                             f"found {len(packet_sections)} sections")

        header = packet_sections[1].split(COMMA_SEPARATOR)
        data = packet_sections[2]

        if len(header) != HEADER_LENGTH:
            raise ValueError(f"Packet header length of {HEADER_LENGTH} expected "
                             f"found {len(header)} instead")

        msg.team_number = int(header[0])
        msg.profile_number = int(header[1])
        msg.profile_half = int(header[2])

        time_data_list: list[float] = []
        depth_data_list: list[float] = []

        for time_reading, depth_reading in [data.split(COMMA_SEPARATOR) for data in
                                            data.split(DATA_SEPARATOR)]:

            if int(time_reading) == 0:
                continue
            # Starts out as uint32
            time_data_list.append(int(time_reading) * MILLISECONDS_TO_SECONDS * SECONDS_TO_MINUTES)

            # Starts out as float
            depth_data_list.append(
                (float(depth_reading) - self.surface_pressure) * MBAR_TO_METER_OF_HEAD)
        msg.time_data = time_data_list
        msg.depth_data = depth_data_list

        return msg


def main() -> None:
    """Run the serial reader node."""
    rclpy.init()

    serial_reader = SerialReader()
    serial_reader.start()
    rclpy.spin(serial_reader)


if __name__ == '__main__':
    main()
