# flooding (boolean) (True is flooding)
from rov_msgs.msg import Flooding
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt6.QtGui import QFont, QColor, QPalette
from gui.event_nodes.subscriber import GUIEventSubscriber
from PyQt6.QtCore import pyqtSignal, pyqtSlot


class FloodVisual(QWidget):

    signal: pyqtSignal = pyqtSignal(Flooding)

    def __init__(self):
        super().__init__()
        # Boilerplate PyQt Setup to link to ROS through a signal/subscriber
        self.signal.connect(self.refresh)
        self.subscription: GUIEventSubscriber = GUIEventSubscriber(Flooding,
                                                                   'flooding',
                                                                   self.signal,
                                                                   10)
        # Create basic 2 vertical stacked boxes layout
        self.layout: QVBoxLayout = QVBoxLayout()
        self.layout.setContentsMargins(0,0,0,0)
        self.layout.setSpacing(20)
        # Create the label that tells us what this is
        self.label: QLabel = QLabel('Flooding Indicator')
        # Set font and size
        font: QFont = QFont("Arial", 14)
        self.label.setFont(font)
        self.layout.addWidget(self.label)
       
        self.indicator: Color = Color("green")
        self.layout.addWidget(self.indicator)
        self.setLayout(self.layout)

    @pyqtSlot(Flooding)
    def refresh(self, msg: Flooding):
        palette = self.palette()
        if Flooding.flooding:
            palette.setColor(QPalette.ColorRole.Window, QColor("red"))
            self.indicator.setPalette(palette)


class Color(QWidget):

    def __init__(self, color):
        super(Color, self).__init__()
        self.setAutoFillBackground(True)

        palette = self.palette()
        palette.setColor(QPalette.ColorRole.Window, QColor(color))
        self.setPalette(palette)
