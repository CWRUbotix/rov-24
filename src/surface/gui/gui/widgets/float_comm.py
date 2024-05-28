from gui.gui_nodes.event_nodes.publisher import GUIEventPublisher
from gui.gui_nodes.event_nodes.subscriber import GUIEventSubscriber
from PyQt6.QtCore import pyqtSignal, pyqtSlot
from PyQt6.QtGui import QTextCursor
from PyQt6.QtWidgets import (QHBoxLayout, QLabel, QPushButton, QTextEdit,
                             QVBoxLayout, QWidget)
from pyqtgraph import PlotWidget

from rov_msgs.msg import FloatCommand, FloatData, FloatSerial


class FloatComm(QWidget):
    """FloatComm widget for sending Float Communication Commands."""

    handle_data_signal = pyqtSignal(FloatData)
    handle_serial_signal = pyqtSignal(FloatSerial)

    def __init__(self) -> None:
        super().__init__()

        layout = QHBoxLayout()
        self.setLayout(layout)

        self.handle_data_signal.connect(self.handle_data)
        self.handle_serial_signal.connect(self.handle_serial)
        GUIEventSubscriber(FloatData, "transceiver_data", self.handle_data_signal)
        GUIEventSubscriber(FloatSerial, "float_serial", self.handle_serial_signal)

        command_pub = GUIEventPublisher(FloatCommand, "float_command")

        info_layout = QVBoxLayout()

        self.team_number = QLabel('Waiting for Team #')
        self.profile_number = QLabel("Waiting for profile #")
        self.profile_half = QLabel("Waitng for profile half")

        left_side_layout = QVBoxLayout()

        info_layout.addWidget(self.team_number)
        info_layout.addWidget(self.profile_number)
        info_layout.addWidget(self.profile_half)

        info_and_buttons = QHBoxLayout()
        info_and_buttons.addLayout(info_layout)

        submerge_button = QPushButton("Submerge")
        pump_button = QPushButton("Pump")
        suck_button = QPushButton("Suck")
        return_button = QPushButton("Return")
        stop_button = QPushButton("Stop")

        submerge_button.clicked.connect(lambda: command_pub.publish(
            FloatCommand(command=FloatCommand.SUBMERGE)
        ))
        pump_button.clicked.connect(lambda: command_pub.publish(
            FloatCommand(command=FloatCommand.PUMP)
        ))
        suck_button.clicked.connect(lambda: command_pub.publish(
            FloatCommand(command=FloatCommand.SUCK)
        ))
        return_button.clicked.connect(lambda: command_pub.publish(
            FloatCommand(command=FloatCommand.RETURN)
        ))
        stop_button.clicked.connect(lambda: command_pub.publish(
            FloatCommand(command=FloatCommand.STOP)
        ))

        info_and_buttons.addWidget(submerge_button)
        info_and_buttons.addWidget(pump_button)
        info_and_buttons.addWidget(suck_button)
        info_and_buttons.addWidget(return_button)
        info_and_buttons.addWidget(stop_button)

        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)

        font = self.console.font()
        font.setFamily("Courier")
        font.setPointSize(11)
        self.console.setFont(font)

        self.plots = [PlotWidget(), PlotWidget()]

        left_side_layout.addLayout(info_and_buttons)
        left_side_layout.addWidget(self.console)

        layout.addLayout(left_side_layout)
        for plot in self.plots:
            layout.addWidget(plot)

        self.time_data: list[float] = []
        self.depth_data: list[float] = []
        self.received_first_half = False
        self.received_second_half = False
        self.completed_profile_one = False

    @pyqtSlot(FloatData)
    def handle_data(self, msg: FloatData) -> None:
        """
        Set the widget label text to the message in the FloatCommand.

        Parameters
        ----------
        msg : FloatData
            the data from the float
        """
        self.team_number.setText(f"Team #: {msg.team_number}")
        self.profile_number.setText(f"Profile #: {msg.profile_number}")
        self.profile_half.setText(f"Profile half: {msg.profile_half}")

        time_data = list(msg.time_data)
        depth_data = list(msg.depth_data)

        if msg.profile_number == 0 and self.completed_profile_one:
            return
        elif msg.profile_number not in (0, 1):
            return

        if msg.profile_half == 0 and not self.received_first_half:
            self.time_data = time_data + self.time_data
            self.depth_data = depth_data + self.depth_data
            self.received_first_half = True
        elif msg.profile_half == 1 and not self.received_second_half:
            self.time_data = self.time_data + time_data
            self.depth_data = self.depth_data + depth_data
            self.received_second_half = True

        if self.received_first_half and self.received_second_half:
            self.plots[msg.profile_number].plot(self.time_data, self.depth_data)
            self.time_data = []
            self.depth_data = []
            self.received_first_half = False
            self.received_second_half = False
            self.completed_profile_one = True

    @pyqtSlot(FloatSerial)
    def handle_serial(self, msg: FloatSerial) -> None:
        """
        Set the widget label text to the message in the FloatCommand.

        Parameters
        ----------
        msg : FloatSerial
            the serial from the float
        """
        self.console.moveCursor(QTextCursor.MoveOperation.End)
        self.console.insertPlainText(f'{msg.serial}\n')
