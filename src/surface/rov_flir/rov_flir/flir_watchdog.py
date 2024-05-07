import atexit
import sys
from subprocess import Popen

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

WATCHDOG_RATE = 10
NAMESPACE = "surface"


class Watchdog():
    def __init__(self, node: Node, args: list[str]) -> None:
        self.node = node
        self.args = args
        self.process: Popen[bytes]

        self.start_process()

    def start_process(self) -> None:
        self.process = Popen(self.args, stdout=sys.stdout, stderr=sys.stderr)

    def poll(self) -> None:
        if self.process.poll() is not None:
            self.node.get_logger().warning("Detected camera crash, restarting...")
            self.start_process()

    def kill(self) -> None:
        self.process.kill()


class FlirWatchdogNode(Node):
    def __init__(self) -> None:
        super().__init__("flir_watchdog_node")

        self.timer = self.create_timer(1 / WATCHDOG_RATE, self.timer_callback)

        self.get_logger().info("Starting camera drivers")

        self.front_watchdog = Watchdog(
            node=self,
            args=["ros2", "launch", "rov_flir", "flir_launch.py",
                  "launch_bottom:=false", f"ns:={NAMESPACE}"]
        )
        self.bottom_watchdog = Watchdog(
            node=self,
            args=["ros2", "launch", "rov_flir", "flir_launch.py",
                  "launch_front:=false", f"ns:={NAMESPACE}"]
        )

        atexit.register(self.front_watchdog.kill)
        atexit.register(self.bottom_watchdog.kill)

    def timer_callback(self) -> None:
        self.front_watchdog.poll()
        self.bottom_watchdog.poll()


def main() -> None:
    rclpy.init()
    flir_watchdog_node = FlirWatchdogNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(flir_watchdog_node, executor=executor)


if __name__ == "__main__":
    main()
