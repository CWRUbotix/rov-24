from gui.gui_nodes.event_nodes.subscriber import GUIEventSubscriber
from PyQt6.QtCore import pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import QHBoxLayout, QLabel, QWidget, QTextEdit
from PyQt6.QtGui import QTextCursor
from qwt.plot import QwtPlot
from qwt.plot_curve import QwtPlotCurve

from rov_msgs.msg import FloatData, FloatSerial, FloatCommand


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

        # TODO add label for team number
        # and maybe profile number for debugging
        # self.label: QLabel = QLabel()
        # self.label.setText('Waiting for radio...')
        # layout.addWidget(self.label)
        self.plot = QwtPlot("Meter of Head Versus Seconds")
        self.plot.setAutoReplot(True)
        self.plot.show()

        layout.addWidget(self.plot)

        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)

        font = self.console.font()
        font.setFamily("Courier")
        font.setPointSize(11)
        self.console.setFont(font)

        layout.addWidget(self.console)

    @pyqtSlot(FloatData)
    def handle_data(self, msg: FloatData) -> None:
        """
        Set the widget label text to the message in the FloatCommand.

        Parameters
        ----------
        msg : FloatData
            the data from the float
        """

        self.plot.make(msg.time_data, msg.depth_data, plot=self.plot)

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
