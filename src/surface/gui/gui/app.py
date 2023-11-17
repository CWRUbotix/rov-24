import atexit
import signal

import qdarktheme
import rclpy
from PyQt6.QtWidgets import QApplication, QWidget
from rclpy.node import Node


class App(QWidget):
    """Main app window."""

    app: QApplication = QApplication([])

    def __init__(self, node_name: str) -> None:
        if not rclpy.ok():
            rclpy.init()
        super().__init__()
        self.node = Node(node_name, parameter_overrides=[])

        self.node.declare_parameter('theme', '')
        self.resize(1850, 720)

        atexit.register(self.clean_shutdown)

    def clean_shutdown(self) -> None:
        if rclpy.ok():
            rclpy.shutdown()

    def run_gui(self) -> None:
        # Kills with Control + C
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        if self.node.get_parameter('theme').get_parameter_value().string_value == "dark":
            qdarktheme.setup_theme()
        elif self.node.get_parameter('theme').get_parameter_value().string_value == "watermelon":
            # UGLY But WORKS
            self.app.setStyleSheet("QWidget { background-color: green; color: pink; }")
        else:
            qdarktheme.setup_theme("light")
        # Delete node now that we've used it to get params
        self.node.destroy_node()

        self.show()

        # TODO: when the app closes it causes an error. Make not cause error?
        self.app.exec()
