from gui.app import App
from gui.widgets.arm import Arm
from gui.widgets.timer import TimerDisplay
from gui.widgets.flood_warning import FloodWarning
from gui.widgets.video_widget import (CameraDescription, CameraType,
                                      SwitchableVideoWidget, VideoWidget)
from gui.widgets.livestream_header import LivestreamHeader
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QHBoxLayout, QVBoxLayout


FRONT_CAM_TOPIC = 'front_cam/image_raw'
BOTTOM_CAM_TOPIC = 'bottom_cam/image_raw'
DEPTH_CAM_TOPIC = 'depth_cam/image_raw'


def make_switchable_video_widget(descriptions: list[CameraDescription]):
    SwitchableVideoWidget(descriptions, "camera_switch")


def make_bottom_bar():
    bottom_screen_layout = QHBoxLayout()

    timer = TimerDisplay()
    bottom_screen_layout.addWidget(timer)

    flood_widget = FloodWarning()
    bottom_screen_layout.addWidget(flood_widget, alignment=Qt.AlignmentFlag.AlignHCenter |
                                   Qt.AlignmentFlag.AlignBottom)

    arm = Arm()
    bottom_screen_layout.addWidget(arm, alignment=Qt.AlignmentFlag.AlignRight |
                                   Qt.AlignmentFlag.AlignBottom)

    return bottom_screen_layout


class PilotApp(App):
    def __init__(self) -> None:
        super().__init__('pilot_gui_node')

        main_layout = QVBoxLayout()
        self.setLayout(main_layout)

        simulation_param = self.node.declare_parameter('simulation', False)
        gui_param = self.node.declare_parameter('gui', 'pilot')

        if simulation_param.value:
            front_cam_type = CameraType.SIMULATION
            bottom_cam_type = CameraType.SIMULATION
            depth_cam_type = CameraType.SIMULATION
        else:
            front_cam_type = CameraType.ETHERNET
            bottom_cam_type = CameraType.ETHERNET
            depth_cam_type = CameraType.DEPTH

        if gui_param.value == 'pilot':
            self.setWindowTitle('Pilot GUI - CWRUbotix ROV 2024')

            front_cam_description = CameraDescription(
                front_cam_type,
                FRONT_CAM_TOPIC,
                "Front Camera",
                1280, 720
            )
            bottom_cam_description = CameraDescription(
                bottom_cam_type,
                BOTTOM_CAM_TOPIC,
                "Bottom Camera",
                1280, 720
            )
            # depth_cam_description = CameraDescription(
            #     depth_cam_type,
            #     DEPTH_CAM_TOPIC,
            #     "Depth Cam",
            #     640, 360
            # )

            main_layout.addWidget(VideoWidget(front_cam_description),
                                  alignment=Qt.AlignmentFlag.AlignHCenter)
            main_layout.addWidget(VideoWidget(bottom_cam_description),
                                  alignment=Qt.AlignmentFlag.AlignHCenter)
            main_layout.addLayout(make_bottom_bar())

        elif gui_param.value == 'livestream':
            top_bar = QHBoxLayout()
            top_bar.addWidget(LivestreamHeader())
            top_bar.addWidget(TimerDisplay(), 2)
            arm_widget = Arm()
            arm_widget.setMaximumWidth(300)
            top_bar.addWidget(arm_widget, 2)

            main_layout.addLayout(top_bar)

            self.setWindowTitle('Livestream GUI - CWRUbotix ROV 2024')

            front_cam_description = CameraDescription(
                front_cam_type,
                FRONT_CAM_TOPIC,
                "Front Camera",
                1280, 720
            )
            bottom_cam_description = CameraDescription(
                bottom_cam_type,
                BOTTOM_CAM_TOPIC,
                "Bottom Camera",
                1280, 720
            )

            video_layout = QHBoxLayout()

            video_layout.addWidget(VideoWidget(front_cam_description),
                                   alignment=Qt.AlignmentFlag.AlignHCenter)
            video_layout.addWidget(VideoWidget(bottom_cam_description),
                                   alignment=Qt.AlignmentFlag.AlignHCenter)

            main_layout.addLayout(video_layout)
            main_layout.addStretch()

        else:
            self.setWindowTitle('Debug GUI - CWRUbotix ROV 2024')

            front_cam_description = CameraDescription(
                front_cam_type,
                FRONT_CAM_TOPIC,
                "Front Camera",
                721, 541
            )
            bottom_cam_description = CameraDescription(
                bottom_cam_type,
                BOTTOM_CAM_TOPIC,
                "Bottom Camera",
                721, 541
            )
            depth_cam_description = CameraDescription(
                depth_cam_type,
                DEPTH_CAM_TOPIC,
                "Depth Cam",
                640, 360
            )

            video_layout = QHBoxLayout()

            video_layout.addWidget(VideoWidget(front_cam_description),
                                   alignment=Qt.AlignmentFlag.AlignHCenter)
            video_layout.addWidget(
                SwitchableVideoWidget([bottom_cam_description, depth_cam_description]),
                alignment=Qt.AlignmentFlag.AlignHCenter
            )

            main_layout.addLayout(video_layout)

            main_layout.addLayout(make_bottom_bar())


def run_gui_pilot() -> None:
    PilotApp().run_gui()
