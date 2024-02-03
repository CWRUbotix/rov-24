from gui.widgets.logger import Logger
from gui.widgets.thruster_tester import ThrusterTester
from gui.widgets.arm import Arm
from PyQt6.QtWidgets import QHBoxLayout, QVBoxLayout, QWidget


class DebugWidget(QWidget):
    def __init__(self) -> None:
        super().__init__()

        right_bar = QVBoxLayout()
        right_bar.addWidget(ThrusterTester())
        right_bar.addWidget(Arm())
        right_bar.addStretch()

        top_pane = QHBoxLayout()
        top_pane.addStretch(2)
        top_pane.addLayout(right_bar)

        root_layout = QVBoxLayout(self)
        root_layout.addLayout(top_pane)
        root_layout.addStretch()
        root_layout.addWidget(Logger())
