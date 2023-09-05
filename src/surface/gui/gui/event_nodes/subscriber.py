import atexit
import re
from threading import Thread

from PyQt6.QtCore import pyqtBoundSignal
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_system_default


class GUIEventSubscriber(Node):
    """Multithreaded subscriber for receiving messages to the GUI."""

    def __init__(self, msg_type: type, topic: str, signal: pyqtBoundSignal,
                 qos_profile: QoSProfile = qos_profile_system_default):
        # Name this node with a sanitized version of the topic
        name: str = f'subscriber{re.sub(r"[^a-zA-Z0-9_]", "_", topic)}'
        super().__init__(name, parameter_overrides=[])

        self.subscription = self.create_subscription(
            msg_type, topic, signal.emit, qos_profile)

        custom_executor = SingleThreadedExecutor()
        custom_executor.add_node(self)
        Thread(target=custom_executor.spin, daemon=True,
               name=f'{name}_spin').start()
        atexit.register(custom_executor.shutdown)
