from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_system_default
from rclpy.time import Time
from rclpy.duration import Duration
from rov_msgs.msg import MissionTimerTick
from rov_msgs.srv import MissionTimerSet


PUBLISH_RATE = 10  # Hz
DEFAULT_DURATION = 15 * 60  # Seconds


class TimerNode(Node):
    def __init__(self) -> None:
        super().__init__("timer_node", parameter_overrides=[])

        self.publisher = self.create_publisher(
            MissionTimerTick, "mission_timer", qos_profile_system_default
        )

        self.set_time_service = self.create_service(
            MissionTimerSet, "set_mission_timer", callback=self.set_time_callback
        )

        self.publisher_timer = self.create_timer(1 / PUBLISH_RATE, self.timer_callback)

        self.time_left = Duration(seconds=DEFAULT_DURATION)
        self.is_running = False

        self.lastTimestamp: Optional[Time] = None

    def do_tick(self):
        timestamp = self.get_clock().now()

        if self.is_running and self.lastTimestamp is not None:
            self.time_left = self.lastTimestamp + self.time_left - timestamp

            if self.time_left < Duration(seconds=0):
                self.is_running = False
                self.time_left = Duration(seconds=0)

        self.lastTimestamp = timestamp

    def publish_tick_message(self) -> None:
        msg = MissionTimerTick()
        msg.is_running = self.is_running
        msg.time_left = self.time_left.to_msg()

        self.publisher.publish(msg)

    def timer_callback(self) -> None:
        self.do_tick()
        self.publish_tick_message()

    def set_time_callback(self, request: MissionTimerSet.Request,
                          response: MissionTimerSet.Response) -> MissionTimerSet.Response:
        self.do_tick()

        if request.set_running:
            self.is_running = request.running

        if request.set_time:
            self.time_left = Duration.from_msg(request.time)
            self.lastTimestamp = None

        self.publish_tick_message()

        response.success = True
        return response


def run_timer() -> None:
    rclpy.init()
    timer_node = TimerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(timer_node, executor=executor)


if __name__ == "__main__":
    run_timer()
