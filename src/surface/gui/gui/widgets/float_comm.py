from PyQt6.QtWidgets import QPushButton, QHBoxLayout, QLabel, QWidget
from gui.gui_nodes.event_nodes.publisher import GUIEventPublisher
from rov_msgs.msg import FloatCommand
from PyQt6.QtCore import pyqtSignal, pyqtSlot


class FloatComm(QWidget):
    """FloatComm widget for sending Float Communication Commands."""

    handle_scheduler_response_signal: pyqtSignal = pyqtSignal(FloatCommand)

    def __init__(self) -> None:
        super().__init__()

        layout: QHBoxLayout = QHBoxLayout()
        self.setLayout(layout)

        submerge_button = QPushButton()
        submerge_button.setText("Submerge")
        submerge_button.setFixedSize(300, 200)
        submerge_button.clicked.connect(self.submerge_clicked)
        layout.addWidget(submerge_button)

        self.handle_scheduler_response_signal.connect(self.handle_text)

        self.label: QLabel = QLabel()
        self.label.setText('Waiting for radio...')
        layout.addWidget(self.label)

        self.transceiver_publisher = GUIEventPublisher(
            FloatCommand,
            "transceiver_control"
        )

    @pyqtSlot(FloatCommand)
    def handle_text(self, msg: FloatCommand) -> None:
        """
        Set the widget label text to the message in the FloatCommand.

        Parameters
        ----------
        msg : FloatCommand
            the command that determines the label text
        """
        self.label.setText(msg.command)

    def submerge_clicked(self) -> None:
        """Publish the command for the float to submerge."""
        self.transceiver_publisher.publish(FloatCommand(command="submerge"))
