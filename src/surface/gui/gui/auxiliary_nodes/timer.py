import typing as t
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_system_default
from rclpy.time import Time
from rclpy.duration import Duration
from rov_msgs.msg import MissionTimer
from rov_msgs.srv import MissionTimerSetTime, MissionTimerSetRunning


PUBLISH_RATE = 10  # Hz
DEFAULT_DURATION = 15 * 60  # Seconds


class TimerNode(Node):
    def __init__(self) -> None:
        super().__init__("timer_node", parameter_overrides=[])

        self.publisher = self.create_publisher(
            MissionTimer, "mission_timer", qos_profile_system_default
        )

        self.set_time_service = self.create_service(
            MissionTimerSetTime, "set_mission_timer", callback=self.set_time_callback
        )
        self.set_running_service = self.create_service(
            MissionTimerSetRunning, "set_mission_timer_running", callback=self.set_running_callback
        )

        self.publisher_timer = self.create_timer(1 / PUBLISH_RATE, self.timer_callback)

        self.time_left = Duration(seconds=DEFAULT_DURATION)
        self.is_running = False

        self.lastTimestamp: t.Optional[Time] = None

    def timer_callback(self) -> None:
        timestamp = self.get_clock().now()

        if self.is_running and self.lastTimestamp is not None:
            self.time_left = self.lastTimestamp + self.time_left - timestamp

            if self.time_left < Duration(seconds=0):
                self.is_running = False
                self.time_left = Duration(seconds=0)

        self.lastTimestamp = timestamp

        msg = MissionTimer()
        msg.is_running = self.is_running
        msg.time_left = self.time_left.to_msg()

        self.publisher.publish(msg)

    def set_time_callback(self, request: MissionTimerSetTime.Request,
                          response: MissionTimerSetTime.Response) -> None:
        self.lastTimestamp = self.get_clock().now()

        self.time_left = Duration.from_msg(request.set_time)

        if request.stop_timer:
            self.is_running = False

        response.is_running = self.is_running

        return response

    def set_running_callback(self, request: MissionTimerSetRunning.Request,
                             response: MissionTimerSetRunning.Response) -> None:
        timestamp = self.get_clock().now()
        if self.is_running and self.lastTimestamp is not None:
            self.time_left = max(
                self.lastTimestamp + self.time_left - timestamp,
                Duration(seconds=0)
            )

        self.lastTimestamp = timestamp

        self.is_running = request.set_running
        response.is_running = self.is_running

        return response


def run_timer() -> None:
    rclpy.init()
    timer_node = TimerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(timer_node, executor=executor)


if __name__ == "__main__":
    run_timer()
