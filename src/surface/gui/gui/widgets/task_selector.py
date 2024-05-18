from gui.gui_nodes.event_nodes.client import GUIEventClient
from PyQt6.QtCore import pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import QGridLayout, QPushButton, QWidget

from rov_msgs.srv import AutonomousFlight

WIDTH = 200


class TaskSelector(QWidget):
    """QWidget that handles task selection with a dropdown."""

    scheduler_response_signal = pyqtSignal(AutonomousFlight.Response)

    def __init__(self) -> None:
        super().__init__()

        self.task_controller = GUIEventClient(AutonomousFlight, 'auto_control_toggle',
                                              self.scheduler_response_signal)

        layout = QGridLayout()
        self.setLayout(layout)

        # Create Start button
        self.start_btn = QPushButton("Start Auto")
        self.start_btn.clicked.connect(lambda: self.task_controller.send_request_async(
                                       AutonomousFlight.Request(
                                           state=AutonomousFlight.Request.START
                                           )))
        self.start_btn.setFixedHeight(75)
        self.start_btn.setFixedWidth(WIDTH)

        # Create Stop button
        self.stop_btn = QPushButton("Stop Auto")
        self.stop_btn.clicked.connect(lambda: self.task_controller.send_request_async(
                                       AutonomousFlight.Request(
                                           state=AutonomousFlight.Request.STOP
                                           )))
        self.stop_btn.setFixedHeight(75)
        self.stop_btn.setFixedWidth(WIDTH)

        # Setup Grid
        layout.addWidget(self.start_btn, 2, 1)
        layout.addWidget(self.stop_btn, 3, 1)

        self.scheduler_response_signal.connect(self.handle_scheduler_response)

    @pyqtSlot(AutonomousFlight.Response)
    def handle_scheduler_response(self, _: AutonomousFlight.Response) -> None:
        """Handle scheduler response to request sent from gui_changed_task."""
        pass
