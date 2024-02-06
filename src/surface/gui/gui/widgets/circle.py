from typing import Callable, Optional, TypeAlias

from gui.styles.custom_styles import WidgetState
from PyQt6.QtCore import QSize, Qt
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import QPushButton, QWidget

QColorConstantsAlias: TypeAlias = QColor | Qt.GlobalColor | int


class CircleButton(QPushButton):
    def __init__(self, func: Callable[[], None],
                 parent: Optional[QWidget] = None,
                 button_label: Optional[str] = None,
                 radius: int = 50,
                 color: Optional[QColorConstantsAlias] = None) -> None:
        super().__init__(parent)

        size = QSize(radius * 2, radius * 2)

        if button_label:
            self.setText(button_label)
        self.setFixedSize(size)
        self.clicked.connect(func)

        if color is None:
            style = f"QWidget {{border-radius : {radius}px;}}"
            self.setStyleSheet(style)
            return

        if isinstance(color, Qt.GlobalColor):
            color = QColor(color)
        elif isinstance(color, int):
            color = QColor(Qt.GlobalColor(color))

        style = ("QWidget {"
                 f"border-radius : {radius}px;"
                 f"background-color: rgb({color.red()}, {color.green()}, {color.blue()})}}")
        self.setStyleSheet(style)


class Circle(CircleButton):
    def __init__(self, parent: Optional[QWidget] = None,
                 button_label: Optional[str] = None,
                 radius: int = 50,
                 color: Optional[QColorConstantsAlias] = None) -> None:
        super().__init__(lambda: None, parent, button_label, radius, color)


class Indicator(Circle):
    def __init__(self, parent: Optional[QWidget] = None,
                 button_label: Optional[str] = None,
                 radius: int = 50) -> None:
        super().__init__(parent, button_label, radius)
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
