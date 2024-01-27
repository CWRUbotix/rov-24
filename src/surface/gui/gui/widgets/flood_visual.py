# flooding (boolean) (True is flooding)
from rov_msgs.msg import Flooding
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt6.QtGui import QFont
from gui.event_nodes.subscriber import GUIEventSubscriber
from PyQt6.QtCore import pyqtSignal, pyqtSlot


class FloodVisual(QWidget):

    signal: pyqtSignal = pyqtSignal(Flooding)

    def __init__(self) -> None:
        super().__init__()
        # Boilerplate PyQt Setup to link to ROS through a signal/subscriber
        self.signal.connect(self.refresh)
        self.subscription: GUIEventSubscriber = GUIEventSubscriber(Flooding,
                                                                   'flooding',
                                                                   self.signal)
        # Create basic 2 vertical stacked boxes layout
        self.mylayout = QVBoxLayout()
        # Create the label that tells us what this is
        self.label: QLabel = QLabel('Flooding Indicator')
        # Set font and size
        font: QFont = QFont("Arial", 14)
        self.label.setFont(font)
        self.mylayout.addWidget(self.label)

        # Text Indicator
        self.indicator: QLabel = QLabel('No Water present')
        self.indicator.setFont(font)
        self.mylayout.addWidget(self.indicator)
        self.setLayout(self.mylayout)

    @pyqtSlot(Flooding)
    def refresh(self, msg: Flooding) -> None:
        if Flooding.flooding:
            self.indicator.setText('FLOODING')
            font: QFont = QFont("Arial", 14)
            self.indicator.setFont(font)
            self.subscription.get_logger().error("Robot is actively flooding, do something!")
