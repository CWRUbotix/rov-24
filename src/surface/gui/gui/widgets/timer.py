from PyQt6.QtCore import Qt, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import QGridLayout, QLabel, QPushButton, QWidget
from rov_msgs.msg import MissionTimer
from rov_msgs.srv import MissionTimerSetTime, MissionTimerSetRunning
from rclpy.duration import Duration

from gui.event_nodes.client import GUIEventClient
from gui.event_nodes.subscriber import GUIEventSubscriber


RESET_SECONDS = 15 * 60  # The number of seconds to set the timer to when reset is clicked


class Timer(QWidget):
    mission_timer_signal = pyqtSignal(MissionTimer)
    set_time_response_signal: pyqtSignal = pyqtSignal(MissionTimerSetTime.Response)
    set_running_response_signal: pyqtSignal = pyqtSignal(MissionTimerSetRunning.Response)

    def __init__(self) -> None:
        super().__init__()

        self.timer_subscription = GUIEventSubscriber(
            MissionTimer,
            'mission_timer',
            self.mission_timer_signal,
        )
        self.mission_timer_signal.connect(self.mission_timer_callback)

        self.set_time_client = GUIEventClient(
            MissionTimerSetTime,
            "set_mission_timer",
            self.set_time_response_signal
        )
        self.set_time_response_signal.connect(self.set_time_response_callback)

        self.set_running_client = GUIEventClient(
            MissionTimerSetRunning,
            "set_mission_timer_running",
            self.set_running_response_signal
        )
        self.set_running_response_signal.connect(self.set_running_response_callback)

        self.running = False

        self.label = QLabel('Label')
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label.setFont(QFont('Arial', 36))
        self.label.setText("--:--")

        self.toggle_btn = QPushButton('Start')
        self.reset_btn = QPushButton('Reset')

        self.toggle_btn.setCheckable(True)

        layout = QGridLayout()
        layout.setContentsMargins(0, 0, 0, 0)

        layout.addWidget(self.label, 0, 0, 1, 2)
        layout.addWidget(self.toggle_btn, 1, 0)
        layout.addWidget(self.reset_btn, 1, 1)

        self.toggle_btn.clicked.connect(self.toggle_timer)
        self.reset_btn.clicked.connect(self.reset_timer)

        self.setLayout(layout)

    def update_label(self, seconds_left: int) -> None:
        minutes = seconds_left // 60
        seconds = seconds_left % 60
        self.label.setText(f"{minutes:02d}:{seconds:02d}")

    def update_buttons(self) -> None:
        self.toggle_btn.setChecked(self.running)
        if self.running:
            self.toggle_btn.setText("Pause")
        else:
            self.toggle_btn.setText("Start")

    def toggle_timer(self) -> None:
        self.set_running_client.send_request_async(
            MissionTimerSetRunning.Request(set_running=not self.running)
        )

    def reset_timer(self) -> None:
        self.set_time_client.send_request_async(
            MissionTimerSetTime.Request(
                set_time=Duration(seconds=RESET_SECONDS).to_msg(),
                stop_timer=True
            )
        )

    @pyqtSlot(MissionTimer)
    def mission_timer_callback(self, msg: MissionTimer) -> None:
        self.running = msg.is_running
        self.update_buttons()

        # Round seconds up, so the timer only displays 0 when the time remaining is actually 0
        seconds_left = msg.time_left.sec + (msg.time_left.nanosec > 0)
        self.update_label(seconds_left)

    @pyqtSlot(MissionTimerSetTime.Response)
    def set_time_response_callback(self, res: MissionTimerSetTime.Response) -> None:
        self.running = res.is_running
        self.update_buttons()

    @pyqtSlot(MissionTimerSetRunning.Response)
    def set_running_response_callback(self, res: MissionTimerSetRunning.Response) -> None:
        self.running = res.is_running
        self.update_buttons()
