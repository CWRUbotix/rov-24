import atexit
import signal
import sys

import qdarktheme
import rclpy
from PyQt6.QtWidgets import QApplication, QWidget
from rclpy.node import Node


class App(Node, QWidget):
    """Main app window."""

    def __init__(self, node_name: str):
        self.app: QApplication = QApplication(sys.argv)
        rclpy.init()

        super().__init__(
            node_name=node_name,
            parameter_overrides=[],
            namespace='surface/gui')
        super(QWidget, self).__init__()

        self.declare_parameter('theme', '')
        self.resize(1850, 720)

        def kill():
            self.destroy_node()
            rclpy.shutdown()

        atexit.register(kill)

    def run_gui(self):
        # Kills with Control + C
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        if self.get_parameter('theme').get_parameter_value().string_value == "dark":
            qdarktheme.setup_theme()
        elif self.get_parameter('theme').get_parameter_value().string_value == "watermelon":
            # UGLY But WORKS
            self.app.setStyleSheet("QWidget { background-color: green; color: pink; }")
        else:
            qdarktheme.setup_theme("light")

        self.show()
        sys.exit(self.app.exec())
