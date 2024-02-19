from PyQt6.QtCore import Qt, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import QGridLayout, QLabel, QPushButton, QWidget
from rov_msgs.msg import MissionTimerTick
from rov_msgs.srv import MissionTimerSet
from rclpy.duration import Duration

from gui.gui_nodes.event_nodes.client import GUIEventClient
from gui.gui_nodes.event_nodes.subscriber import GUIEventSubscriber


RESET_SECONDS = 15 * 60  # The number of seconds to set the timer to when reset is clicked


class TimerBase(QWidget):
    mission_timer_signal = pyqtSignal(MissionTimerTick)

    def __init__(self) -> None:
        super().__init__()

        self.timer_subscription = GUIEventSubscriber(
            MissionTimerTick,
            'mission_timer',
            self.mission_timer_signal,
        )
        self.mission_timer_signal.connect(self.mission_timer_callback)

        self.running = False

        self.label = QLabel('Label')
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label.setFont(QFont('Arial', 36))
        self.label.setText("--:--")

    def update_label(self, seconds_left: int) -> None:
        minutes = seconds_left // 60
        seconds = seconds_left % 60
        self.label.setText(f"{minutes:02d}:{seconds:02d}")

    @pyqtSlot(MissionTimerTick)
    def mission_timer_callback(self, msg: MissionTimerTick) -> None:
        self.running = msg.is_running

        # Round seconds up, so the timer only displays 0 when the time remaining is actually 0
        seconds_left = msg.time_left.sec + (msg.time_left.nanosec > 0)
        self.update_label(seconds_left)


class DisplayTimer(TimerBase):
    def __init__(self) -> None:
        super().__init__()

        layout = QGridLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.label)

        self.setLayout(layout)


class InteractiveTimer(TimerBase):
    set_timer_response_signal: pyqtSignal = pyqtSignal(MissionTimerSet.Response)

    def __init__(self) -> None:
        super().__init__()

        self.toggle_btn = QPushButton('Start')
        self.reset_btn = QPushButton('Reset')

        self.set_timer_client = GUIEventClient(
            MissionTimerSet,
            "set_mission_timer",
            self.set_timer_response_signal
        )
        self.set_timer_response_signal.connect(self.set_time_response_callback)

        self.toggle_btn.setCheckable(True)

        self.toggle_btn.clicked.connect(self.toggle_timer)
        self.reset_btn.clicked.connect(self.reset_timer)

        layout = QGridLayout()
        layout.setContentsMargins(0, 0, 0, 0)

        layout.addWidget(self.label, 0, 0, 1, 2)
        layout.addWidget(self.toggle_btn, 1, 0)
        layout.addWidget(self.reset_btn, 1, 1)

        self.setLayout(layout)

    def update_buttons(self) -> None:
        self.toggle_btn.setChecked(self.running)
        if self.running:
            self.toggle_btn.setText("Pause")
        else:
            self.toggle_btn.setText("Start")

    def mission_timer_callback(self, msg: MissionTimerTick) -> None:
        super().mission_timer_callback(msg)
        self.update_buttons()

    def toggle_timer(self) -> None:
        self.set_timer_client.send_request_async(
            MissionTimerSet.Request(
                set_running=True,
                running=not self.running
            )
        )

    def reset_timer(self) -> None:
        self.set_timer_client.send_request_async(
            MissionTimerSet.Request(
                set_time=True,
                time=Duration(seconds=RESET_SECONDS).to_msg(),
                set_running=True,
                running=False
            )
        )

    @pyqtSlot(MissionTimerSet.Response)
    def set_time_response_callback(self, res: MissionTimerSet.Response) -> None:
        if not res.success:
            self.set_timer_client.get_logger().error("Failed to set timer")
