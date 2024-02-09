from typing import Callable, Optional
from gui.styles.custom_styles import WidgetStateInterface
from PyQt6.QtCore import QSize, Qt
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import QPushButton, QWidget, QLabel


class CircleInterface(QWidget):
    def __init__(self, parent: Optional[QWidget] = None, radius: int = 50) -> None:
        super().__init__(parent)

        self.setFixedSize(QSize(2 * radius, 2 * radius))
        stylesheet = self.styleSheet()
        self.setStyleSheet(f"{stylesheet}border-radius: {radius}px;")

    def set_color(self, color: QColor | Qt.GlobalColor) -> None:
        if isinstance(color, Qt.GlobalColor):
            color = QColor(color)
        style = f"background-color: rgb({color.red()}, {color.green()}, {color.blue()});"
        self.setStyleSheet(f"{self.styleSheet()}{style}")


class CircleButton(QPushButton, CircleInterface):
    def __init__(self, func: Callable[[], None],
                 parent: Optional[QWidget] = None,
                 button_label: Optional[str] = None,
                 radius: int = 50,
                 color: Optional[QColor | Qt.GlobalColor] = None) -> None:
        super().__init__(parent)
        CircleInterface.__init__(self, parent, radius)

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
        CircleInterface.__init__(self, parent, radius)

        if color:
            self.set_color(color)


class Indicator(Circle, WidgetStateInterface):
    def __init__(self, parent: Optional[QWidget] = None,
                 radius: int = 50) -> None:
        super().__init__(parent, radius)
        self.set_inactive()

    def good_state(self) -> None:
        self.set_on()

    def bad_state(self) -> None:
        self.set_off()
