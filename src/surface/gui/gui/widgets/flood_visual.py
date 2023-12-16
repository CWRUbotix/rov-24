## /flooding (boolean) (True is flooding)
from rov_msgs.msg import Flooding
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt6.QtGui import QFont
from gui.event_nodes.subscriber import GUIEventSubscriber
from PyQt6.QtCore import pyqtSignal, pyqtSlot

class FloodVisual(QWidget):
    signal: pyqtSignal = pyqtSignal(Flooding)

    def __init__(self):
        super().__init__()
        self.signal.connect(self.refresh)
        self.subscription: GUIEventSubscriber = GUIEventSubscriber(Flooding, '/flooding', self.signal, 10)
        self.layout = QVBoxLayout()
        self.label = QLabel('Flooding Indicator')
        self.layout.addWidget(self.label)
        self.indicator = QLabel("🟢")
        font = QFont("Arial", 40)  # Set font and size
        self.indicator.setFont(font)
        self.layout.addWidget(self.indicator)        
        self.setLayout(self.layout)

    @pyqtSlot(Flooding)
    def refresh(self, msg: Flooding):
        if Flooding.flooding:
            self.indicator.setText("🔴")

        
