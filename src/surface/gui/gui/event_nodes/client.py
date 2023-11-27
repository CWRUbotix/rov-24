import re
from threading import Thread

import rclpy
from PyQt6.QtCore import pyqtBoundSignal
from rclpy.client import Client, SrvType, SrvTypeRequest
from rclpy.node import Node

# Set to None for no timeout limits on service requests
# else set to float number of seconds to limit request spinning
# TODO? Should be param of each Client
TIMEOUT_SEC: float = 1.0


class GUIEventClient(Node):
    """Multithreaded client for sending service requests from the GUI."""

    def __init__(self, srv_type: SrvType, topic: str, signal: pyqtBoundSignal):
        # Name this node with a sanitized version of the topic
        self.name: str = f'client_{re.sub(r"[^a-zA-Z0-9_]", "_", topic)}'
        super().__init__(self.name, parameter_overrides=[])

        self.srv_type = srv_type
        self.topic: str = topic
        self.signal: pyqtBoundSignal = signal

        self.cli: Client = self.create_client(srv_type, topic)
        Thread(target=self.__connect_to_service, daemon=True,
               name=f'{self.name}_connect_to_service').start()

    def __connect_to_service(self) -> None:
        """Connect this client to a server in a separate thread."""
        while not self.cli.wait_for_service(timeout_sec=TIMEOUT_SEC):
            # TODO this f strings looks janky
            self.get_logger().info(
                'Service for GUI event client node on topic' +
                f' {self.topic} unavailable, waiting again...')

    def send_request_async(self, request: SrvTypeRequest) -> None:
        """Send request to server in separate thread."""
        Thread(target=self.__send_request_with_signal,
               kwargs={'request': request},
               daemon=True, name=f'{self.name}_send_request').start()

    def __send_request_with_signal(self, request: SrvTypeRequest) -> None:
        """Send synchronous request to server and emit signal."""
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=TIMEOUT_SEC)

        try:
            self.signal.emit(future.result())
        except TypeError as exception:
            self.get_logger().error("Request received no response.")
            self.get_logger().error(str(exception))
