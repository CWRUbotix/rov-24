from typing import Callable, Optional

from gui.styles.custom_styles import WidgetState
from PyQt6.QtCore import QSize, Qt
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import QPushButton, QWidget, QLabel
from rclpy.logging import get_logger


class CircleInterface:
    def make_circular(self) -> None:
        if isinstance(self, QWidget):
            if self.size().height() != self.size().width():
                get_logger("CircleInterface").warning("Warning the QWidget is not square shape.")
                get_logger("CircleInterface").warning(str(self.size()))

            radius = self.size().height() / 2
            stylesheet = self.styleSheet()
            self.setStyleSheet(f"{stylesheet}border-radius: {radius}px;")
        else:
            get_logger("CircleInterface").error("Trying to extend circle interface without being a QWidget.")

    def set_color(self, color: QColor | Qt.GlobalColor) -> None:
        if isinstance(self, QWidget):
            if isinstance(color, Qt.GlobalColor):
                color = QColor(color)
            style = f"background-color: rgb({color.red()}, {color.green()}, {color.blue()});"
            self.setStyleSheet(f"{self.styleSheet()}{style}")
        else:
            get_logger("CircleInterface").error("Trying to call set_color without being a QWidget.")


class CircleButton(QPushButton, CircleInterface):
    def __init__(self, func: Callable[[], None],
                 parent: Optional[QWidget] = None,
                 button_label: Optional[str] = None,
                 radius: int = 50,
                 color: Optional[QColor | Qt.GlobalColor] = None) -> None:
        super().__init__(parent)
        size = QSize(radius * 2, radius * 2)
        self.setFixedSize(size)

        self.make_circular()

        if button_label:
            self.setText(button_label)
        self.clicked.connect(func)
        if color:
            self.set_color(color)


class Circle(QLabel, CircleInterface):
    def __init__(self, parent: Optional[QWidget] = None,
                 radius: int = 50,
                 color: Optional[QColor | Qt.GlobalColor] = None) -> None:
        super().__init__(parent)
        size = QSize(radius * 2, radius * 2)
        self.setFixedSize(size)

        self.make_circular()

        if color:
            self.set_color(Qt.GlobalColor.cyan)


class Indicator(Circle):
    def __init__(self, parent: Optional[QWidget] = None,
                 radius: int = 50) -> None:
        super().__init__(parent, radius)
        self.setProperty(WidgetState.PROPERTY_NAME, WidgetState.INACTIVE)

    def good_state(self) -> None:
        self.setProperty(WidgetState.PROPERTY_NAME, WidgetState.ON)
        style = self.style()
        if style is not None:
            style.polish(self)

    def bad_state(self) -> None:
        self.setProperty(WidgetState.PROPERTY_NAME, WidgetState.OFF)
        style = self.style()
        if style is not None:
            style.polish(self)
