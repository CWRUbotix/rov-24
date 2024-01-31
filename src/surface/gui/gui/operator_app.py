from gui.app import App
from gui.widgets.logger import Logger
from gui.widgets.seagrass import SeagrassWidget
from gui.widgets.task_selector import TaskSelector
from gui.widgets.timer import Timer
from PyQt6.QtWidgets import QGridLayout, QTabWidget, QWidget


class OperatorApp(App):
    def __init__(self) -> None:
        super().__init__('operator_gui_node')

        self.setWindowTitle('Operator GUI - CWRUbotix ROV 2024')

        tabs = QTabWidget()

        # Main tab
        main_tab = QWidget()
        main_layout = QGridLayout()
        main_tab.setLayout(main_layout)

        self.timer = Timer()
        main_layout.addWidget(self.timer, 0, 1)

        self.task_selector = TaskSelector()
        main_layout.addWidget(self.task_selector, 1, 1)

        self.logger = Logger()
        main_layout.addWidget(self.logger, 1, 0)

        # Add tabs to root
        root_layout = QGridLayout()
        self.setLayout(root_layout)

        tabs.addTab(main_tab, "Main")
        tabs.addTab(SeagrassWidget(), "Seagrass")

        root_layout.addWidget(tabs)


def run_gui_operator() -> None:
    OperatorApp().run_gui()
