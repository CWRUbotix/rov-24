from PyQt6.QtWidgets import QVBoxLayout, QHBoxLayout, QWidget
from gui.widgets.logger import Logger
from gui.widgets.thruster_tester import ThrusterTester


class DebugWidget(QWidget):
    def __init__(self):
        super().__init__()

        root_layout: QVBoxLayout = QVBoxLayout(self)

        top_pane: QHBoxLayout = QHBoxLayout()
        top_pane.addStretch(2)
        top_pane.addWidget(ThrusterTester())

        root_layout.addLayout(top_pane)
        root_layout.addStretch()
        root_layout.addWidget(Logger())
