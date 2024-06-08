from gui.app import App
from gui.widgets.logger import Logger
from gui.widgets.tabs.general_debug_tab import GeneralDebugTab
from gui.widgets.float_comm import FloatComm
from gui.widgets.timer import InteractiveTimer
from gui.widgets.task_selector import TaskSelector
from gui.widgets.flood_warning import FloodWarning
from PyQt6.QtWidgets import QTabWidget, QWidget, QVBoxLayout, QHBoxLayout


class OperatorApp(App):
    def __init__(self) -> None:
        super().__init__('operator_gui_node')

        self.setWindowTitle('Operator GUI - CWRUbotix ROV 2024')

        # Main tab
        main_tab = QWidget()
        main_layout = QHBoxLayout()
        main_tab.setLayout(main_layout)

        left_pane = QVBoxLayout()
        right_pane = QVBoxLayout()

        main_layout.addLayout(left_pane)
        main_layout.addLayout(right_pane)

        self.float_comm: FloatComm = FloatComm()
        left_pane.addWidget(self.float_comm)

        logger = Logger()
        left_pane.addWidget(logger)

        timer = InteractiveTimer()
        right_pane.addWidget(timer)

        flood_warning = FloodWarning()
        right_pane.addWidget(flood_warning)

        right_pane.addStretch()

        task_selector = TaskSelector()
        right_pane.addWidget(task_selector)

        # Add tabs to root
        root_layout = QVBoxLayout()
        self.setLayout(root_layout)

        tabs = QTabWidget()
        tabs.addTab(main_tab, "Main")
        tabs.addTab(GeneralDebugTab(), "General Debug")
        root_layout.addWidget(tabs)


def run_gui_operator() -> None:
    OperatorApp().run_gui()
