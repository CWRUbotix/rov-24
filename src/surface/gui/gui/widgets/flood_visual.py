## /flooding (boolean) (True is flooding)
import rclpy
from rov_msgs.msg import Flooding
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt5.QtGui import QPainter, QColor
from gui.event_nodes.subscriber import GUIEventSubscriber
from PyQt6.QtCore import pyqtSignal

class FloodVisual(QWidget):
    def __init__(self):
        super().__init__()
        
        self.signal: pyqtSignal = pyqtSignal(Flooding.flooding)
        self.signal.connect(self.refresh)
        self.subscription: GUIEventSubscriber = GUIEventSubscriber(Flooding, '/flooding', signal, 10)

        self.indicator = IndicatorWidget(self)
        self.layout = QVBoxLayout()
        self.label = QLabel('Indicator')
        self.layout.addWidget(self.label)
        self.layout.addWidget(self.indicator)
        self.setLayout(self.layout)

    def refresh(self, msg):
        self.indicator.set_status(msg.flooding)

class IndicatorWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.status = False

    def set_status(self, status):
        self.status = status
        self.update()  # Redraw the widget

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        rect = self.contentsRect()
        color = QColor(0, 255, 0) if self.status else QColor(255, 0, 0)

        # Draw the circular light
        painter.setBrush(color)
        painter.drawEllipse(rect)
        painter.end()