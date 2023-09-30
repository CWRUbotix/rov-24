
import rclpy
from std_msgs.msg import Bool
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt5.QtGui import QColor, QPalette

## /flooding (boolean) (True is flooding)


class FloodVisual(QWidget):
    def __init__(self):
        super().__init__()

        rclpy.init()
        self.node = rclpy.create_node('flood_warning')
        self.subscription = self.node.create_subscription(Bool, '/flooding', self.callback, 10)
        
        # GUI initialization
        self.init_ui()
        
    def init_ui(self):
        self.layout = QVBoxLayout()
        self.label = QLabel('Indicator')
        self.layout.addWidget(self.label)
        self.set_palette_color(QColor(255, 0, 0))  # Default color is red
        self.setLayout(self.layout)

    def set_palette_color(self, color):
        palette = QPalette()
        palette.setColor(QPalette.Window, color)
        self.label.setAutoFillBackground(True)
        self.label.setPalette(palette)

    def callback(self, msg):
        if msg.data:
            self.set_palette_color(QColor(0, 255, 0))  # Green when the topic is true
        else:
            self.set_palette_color(QColor(255, 0, 0))  # Red when the topic is false