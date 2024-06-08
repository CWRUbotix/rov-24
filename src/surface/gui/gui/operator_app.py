from gui.app import App
from gui.widgets.logger import Logger
from gui.widgets.tabs.general_debug_tab import GeneralDebugTab
from gui.widgets.float_comm import FloatComm
from gui.widgets.timer import InteractiveTimer
from gui.widgets.task_selector import TaskSelector
from gui.widgets.temperature import TemperatureSensor
from gui.widgets.heartbeat import HeartbeatWidget
from gui.widgets.ip_widget import IPWidget
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QGridLayout, QTabWidget, QWidget, QVBoxLayout


class OperatorApp(App):
    def __init__(self) -> None:
        super().__init__('operator_gui_node')

        self.setWindowTitle('Operator GUI - CWRUbotix ROV 2024')

        # Main tab
        main_tab = QWidget()
        main_layout = QGridLayout()
        main_tab.setLayout(main_layout)

        right_bar = QVBoxLayout()
        main_layout.addLayout(right_bar, 0, 1)

        timer = InteractiveTimer()
        right_bar.addWidget(timer)

        temp_sensor = TemperatureSensor()
        right_bar.addWidget(temp_sensor)

        right_bar.addWidget(HeartbeatWidget(), alignment=Qt.AlignmentFlag.AlignTop |
                            Qt.AlignmentFlag.AlignLeft)

        right_bar.addWidget(IPWidget(), alignment=Qt.AlignmentFlag.AlignTop |
                            Qt.AlignmentFlag.AlignLeft)

        task_selector = TaskSelector()
        main_layout.addWidget(task_selector, 1, 1)

        self.float_comm: FloatComm = FloatComm()
        main_layout.addWidget(self.float_comm, 0, 0)

        logger = Logger()
        main_layout.addWidget(logger, 1, 0)

        # Add tabs to root
        root_layout = QVBoxLayout()
        self.setLayout(root_layout)

        tabs = QTabWidget()
        tabs.addTab(main_tab, "Main")
        tabs.addTab(GeneralDebugTab(), "General Debug")
        root_layout.addWidget(tabs)


def run_gui_operator() -> None:
    OperatorApp().run_gui()
