import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from serial import Serial

from rov_msgs.msg import FloatData, FloatCommand, FloatSerial

MILLISECONDS_TO_SECONDS = 1/1000
SECONDS_TO_MINUTES = 1/60
MBAR_TO_METER_OF_HEAD = 0.010199773339984


ROS_PACKET = "ROS:"
SECTION_SEPARATOR = ":"
DATA_SEPARATOR = ";"
COMMA_SEPARATOR = ","


class SerialReader(Node):

    def __init__(self) -> None:
        super().__init__('serial_reader')
        self.data_publisher = self.create_publisher(FloatData, 'transceiver_data',
                                               QoSPresetProfiles.SENSOR_DATA.value)

        self.create_subscription(FloatCommand, 'float_command', self.send_command,
                                 QoSPresetProfiles.DEFAULT.value)

        self.serial_publisher = self.create_publisher(FloatSerial, 'float_serial',
                                                      QoSPresetProfiles.SENSOR_DATA.value)
        timer_period = .5

        self.first_attempt = True
        self.create_timer(timer_period, self.timer_callback)
        self.serial = Serial("dev/ttyTransceiver", 115200)

    def send_command(self, msg: FloatCommand) -> None:
        self.serial.write(msg.command.encode())

    def timer_callback(self) -> None:
        """Publish a message from the transceiver."""

        packet = self.serial.readline().decode()

        self.serial_publisher.publish(FloatSerial(serial=packet))

        if ROS_PACKET not in packet:
            return

        msg = FloatData()

        packet_sections = packet.split(SECTION_SEPARATOR)
        header = packet_sections[1]
        data = packet_sections[2]

        msg.team_number = int(header[0])
        msg.profile_number = int(header[1])
        msg.profile_half = int(header[2])

        time_data_list: list[float] = []
        depth_data_list: list[float] = []
        for time_data, depth_data in [data.split(COMMA_SEPARATOR) for data in
                                      data.split(DATA_SEPARATOR)]:
            # Starts out as float32
            time_data_list.append(float(time_data) * MILLISECONDS_TO_SECONDS * SECONDS_TO_MINUTES)

            # Starts out as unsigned long
            depth_data_list.append(int(depth_data) * MBAR_TO_METER_OF_HEAD)

        msg.time_data = time_data_list
        msg.depth_data = depth_data_list
        self.data_publisher.publish(msg)


def main() -> None:
    """Run the serial reader node."""
    rclpy.init()

    serial_reader = SerialReader()

    rclpy.spin(serial_reader)


if __name__ == '__main__':
    main()
