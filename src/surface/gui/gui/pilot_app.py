from gui.app import App
from gui.widgets.arm import Arm
from gui.widgets.video_widget import (CameraType, SwitchableVideoWidget,
                                      VideoWidget)
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QHBoxLayout


class PilotApp(App):
    def __init__(self) -> None:
        super().__init__('pilot_gui_node')

        self.setWindowTitle('Pilot GUI - CWRUbotix ROV 2024')

        layout: QHBoxLayout = QHBoxLayout()
        self.setLayout(layout)

        self.main_video = VideoWidget("front_cam/image_raw", CameraType.ETHERNET, "Front Camera")

        self.video_area = SwitchableVideoWidget(["bottom_cam/image_raw",
                                                 "camera/color/image_raw"],
                                                [CameraType.ETHERNET, CameraType.ETHERNET],
                                                ["Bottom Camera",
                                                 "Depth Camera"],
                                                "camera_switch")
        layout.addWidget(self.main_video, alignment=Qt.AlignmentFlag.AlignLeft)
        layout.addWidget(self.video_area, alignment=Qt.AlignmentFlag.AlignRight)

        self.arm: Arm = Arm()
        layout.addWidget(self.arm,
                         alignment=Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignBottom)


def run_gui_pilot() -> None:
    PilotApp().run_gui()
