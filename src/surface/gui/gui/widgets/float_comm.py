from gui.gui_nodes.event_nodes.subscriber import GUIEventSubscriber
from PyQt6.QtCore import pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import QHBoxLayout, QLabel, QWidget
from qwt.plot import QwtPlot
from qwt.plot_curve import QwtPlotCurve

from rov_msgs.msg import FloatData


class FloatComm(QWidget):
    """FloatComm widget for sending Float Communication Commands."""

    handle_scheduler_response_signal = pyqtSignal(FloatData)

    def __init__(self) -> None:
        super().__init__()

        layout = QHBoxLayout()
        self.setLayout(layout)

        self.handle_scheduler_response_signal.connect(self.handle_data)
        GUIEventSubscriber(FloatData, "transceiver_data", self.handle_scheduler_response_signal)

        # TODO add label for team number
        # and maybe profile number for debugging
        # self.label: QLabel = QLabel()
        # self.label.setText('Waiting for radio...')
        # layout.addWidget(self.label)
        self.plot = QwtPlot("Meter of Head Versus Seconds")
        self.plot.setAutoReplot(True)
        self.plot.show()
        layout.addWidget(self.plot)

    @pyqtSlot(FloatData)
    def handle_data(self, msg: FloatData) -> None:
        """
        Set the widget label text to the message in the FloatCommand.

        Parameters
        ----------
        msg : FloatData
            the data from the float
        """

        QwtPlotCurve.make(msg.time_data, msg.depth_data, plot=self.plot)
