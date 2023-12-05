import atexit
import signal

import qdarktheme
import rclpy.utilities
from PyQt6.QtWidgets import QApplication, QWidget
from rclpy.node import Node


class App(QWidget):
    """Main app window."""

    app: QApplication = QApplication([])

    def __init__(self, node_name: str) -> None:
        if not rclpy.utilities.ok():
            rclpy.init()
        super().__init__()
        self.node = Node(node_name, parameter_overrides=[])

        self.theme_param = self.node.declare_parameter('theme', '')
        self.resize(1850, 720)

        atexit.register(clean_shutdown)

    def run_gui(self) -> None:
        # Kills with Control + C
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        if self.theme_param.get_parameter_value().string_value == "dark":
            qdarktheme.setup_theme()
        elif self.theme_param.get_parameter_value().string_value == "watermelon":
            # UGLY But WORKS
            self.app.setStyleSheet("QWidget { background-color: green; color: pink; }")
        else:
            qdarktheme.setup_theme("light")
        # Delete node now that we've used it to get params
        self.node.destroy_node()

        self.show()

        # TODO: when the app closes it causes an error. Make not cause error?
        self.app.exec()


def clean_shutdown() -> None:
    if rclpy.utilities.ok():
        rclpy.shutdown()
