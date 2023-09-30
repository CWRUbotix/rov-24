## /flooding (boolean) (True is flooding)
from rov_msgs.msg import Flooding
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt6.QtGui import QPainter, QColor
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

        # self.indicator = Indicator()
        # self.layout.addWidget(self.indicator)
        
        self.setLayout(self.layout)

    @pyqtSlot(Flooding)
    def refresh(self, msg: Flooding):
        self.indicator.setBooleanValue(msg.flooding)

# class Indicator(QWidget):
#     def __init__(self, parent=None):
#         super().__init__(parent)
#         self.setFixedSize(100, 100)
#         self.boolean_value = False
#         self.update() 

#     def setBooleanValue(self, value):
#         self.boolean_value = value
#         self.update()  # Trigger a repaint when the boolean value changes

#     def update(self):
#         painter = QPainter(self)
#         painter.begin(self)
#         painter.setRenderHint(QPainter.RenderHint.Antialiasing)

#         # Set the circle color based on the boolean value
#         if self.boolean_value:
#             circle_color = QColor(0, 255, 0)  # Green
#         else:
#             circle_color = QColor(255, 0, 0)  # Red

#         # Draw the colored circle
#         painter.setBrush(circle_color)
#         painter.drawEllipse(self.rect())
#         painter.end()

        

