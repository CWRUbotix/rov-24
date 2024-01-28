from gui.app import App
from gui.widgets.arm import Arm
from gui.widgets.flood_warning import FloodWarning
from gui.widgets.video_widget import CameraType, SwitchableVideoWidget, VideoWidget
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QHBoxLayout, QVBoxLayout, QWidget


class PilotApp(App):
    def __init__(self) -> None:
        super().__init__('pilot_gui_node')

        self.setWindowTitle('Pilot GUI - CWRUbotix ROV 2024')

        main_layout = QVBoxLayout()
        video_layout = QHBoxLayout()
        self.setLayout(main_layout)

        main_video = VideoWidget("front_cam/image_raw", CameraType.ETHERNET, "Front Camera")

        video_area = SwitchableVideoWidget(["bottom_cam/image_raw",
                                            "camera/color/image_raw"],
                                           [CameraType.ETHERNET, CameraType.DEPTH],
                                           ["Bottom Camera", "Depth Camera"],
                                           "camera_switch")

        video_layout.addWidget(main_video, alignment=Qt.AlignmentFlag.AlignHCenter)
        video_layout.addWidget(video_area, alignment=Qt.AlignmentFlag.AlignHCenter)

        main_layout.addLayout(video_layout)

        bottom_screen_layout = QHBoxLayout()

        place_holder = QWidget()
        bottom_screen_layout.addWidget(place_holder)

        flood_widget = FloodWarning()
        bottom_screen_layout.addWidget(flood_widget, alignment=Qt.AlignmentFlag.AlignHCenter |
                                       Qt.AlignmentFlag.AlignBottom)

        arm = Arm()
        bottom_screen_layout.addWidget(arm, alignment=Qt.AlignmentFlag.AlignRight |
                                       Qt.AlignmentFlag.AlignBottom)

        main_layout.addLayout(bottom_screen_layout)


def run_gui_pilot() -> None:
    PilotApp().run_gui()
