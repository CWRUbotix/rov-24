# flooding (boolean) (True is flooding)
from rov_msgs.msg import Flooding
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt6.QtGui import QFont
from gui.event_nodes.subscriber import GUIEventSubscriber
from PyQt6.QtCore import pyqtSignal, pyqtSlot


class FloodWarning(QWidget):

    signal: pyqtSignal = pyqtSignal(Flooding)

    def __init__(self) -> None:
        super().__init__()
        # Boilerplate PyQt Setup to link to ROS through a signal/subscriber
        self.signal.connect(self.refresh)
        self.subscription: GUIEventSubscriber = GUIEventSubscriber(Flooding,
                                                                   'flooding',
                                                                   self.signal)
        # Create a latch variable
        self.latch: bool = False
        # Create basic 2 vertical stacked boxes layout
        self.flood_layour = QVBoxLayout()
        # Create the label that tells us what this is
        self.label: QLabel = QLabel('Flooding Indicator')
        font: QFont = QFont("Arial", 14)
        self.label.setFont(font)
        self.flood_layour.addWidget(self.label)

        self.indicator: QLabel = QLabel('No Water present')
        self.indicator.setFont(font)
        self.flood_layour.addWidget(self.indicator)
        self.setLayout(self.flood_layour)

    @pyqtSlot(Flooding)
    def refresh(self, msg: Flooding) -> None:
        if Flooding.flooding:
            self.indicator.setText('FLOODING')
            self.subscription.get_logger().error("Robot is actively flooding, do something!")
            self.latch = True
        else:
            self.indicator.setText('No Water present')
            if self.latch:
                self.subscription.get_logger().warning("Robot flooding has reset itself.")
                self.latch = False
