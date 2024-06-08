from gui.gui_nodes.event_nodes.subscriber import GUIEventSubscriber
from rov_msgs.msg import Temperature
from PyQt6.QtCore import pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import (QGridLayout, QLabel, QLineEdit, QPushButton,
                             QVBoxLayout, QWidget)

class ThrusterTester(QWidget):


    temperature_reading_signal: pyqtSignal = pyqtSignal()

    def __init__(self) -> None:
        self.cmd_client: GUIEventSubscriber = GUIEventSubscriber(
            Temperature,
            "temperature",
            self.temperature_reading_signal
        )
