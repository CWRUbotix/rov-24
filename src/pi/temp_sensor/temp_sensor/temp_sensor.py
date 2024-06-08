import rclpy
from rclpy.node import Node
from rov_msgs.msg import Temperature

import tsys01


class TempSensor(Node):

    def __init__(self) -> None:
        super().__init__('temp_sensor', parameter_overrides=[])
        self.publisher = self.create_publisher(Temperature, 'temperature', 10)
        self.sensor = tsys01.TSYS01()
        self.sensor.init()

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self) -> None:
        try:
            self.sensor.read()
            temp_reading = self.sensor.temperature()

            # If any of the sensors detect water, send true to /tether/flooding
            msg = Temperature()
            msg.reading = temp_reading
            self.publisher.publish(msg)
        except OSError:
            print('Failed to read temperature, skipping this read')


def main(args: None = None) -> None:
    rclpy.init()
    temp_sensor = TempSensor()
    rclpy.spin(temp_sensor)
    temp_sensor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
