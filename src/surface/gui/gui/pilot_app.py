from gui.app import App
from gui.widgets.arm import Arm
from gui.widgets.timer import TimerDisplay
from gui.widgets.flood_warning import FloodWarning
from gui.widgets.video_widget import (CameraDescription, CameraType,
                                      SwitchableVideoWidget, VideoWidget)
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QHBoxLayout, QVBoxLayout, QBoxLayout


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
            title = 'Pilot GUI - CWRUbotix ROV 2024'

            video_layout: QHBoxLayout | QBoxLayout = main_layout
            front_cam_size = [1280, 720]
            bottom_cam_size = front_cam_size
            depth_cam_size = [640, 360]
        else:
            title = 'Debug GUI - CWRUbotix ROV 2024'

            video_layout = QHBoxLayout()
            main_layout.addLayout(video_layout)
            front_cam_size = [721, 541]
            bottom_cam_size = front_cam_size
            depth_cam_size = [640, 360]

        # 1 big camera feed and 2 smaller ones
        front_cam_description = CameraDescription(front_cam_type,
                                                  'front_cam/image_raw',
                                                  'Front Camera', *front_cam_size)

        bottom_cam_description = CameraDescription(bottom_cam_type,
                                                   'bottom_cam/image_raw',
                                                   'Bottom Camera', *bottom_cam_size)
        depth_cam_description = CameraDescription(depth_cam_type,
                                                  'depth_cam/image_raw',
                                                  'Depth Camera', *depth_cam_size)

        main_video = VideoWidget(front_cam_description)
        video_area = SwitchableVideoWidget([bottom_cam_description, depth_cam_description],
                                           "camera_switch")

        self.setWindowTitle(title)
        video_layout.addWidget(main_video, alignment=Qt.AlignmentFlag.AlignHCenter)
        video_layout.addWidget(video_area, alignment=Qt.AlignmentFlag.AlignHCenter)

        bottom_screen_layout = QHBoxLayout()

        timer = TimerDisplay()
        bottom_screen_layout.addWidget(timer)

        flood_widget = FloodWarning()
        bottom_screen_layout.addWidget(flood_widget, alignment=Qt.AlignmentFlag.AlignHCenter |
                                       Qt.AlignmentFlag.AlignBottom)

        arm = Arm()
        bottom_screen_layout.addWidget(arm, alignment=Qt.AlignmentFlag.AlignRight |
                                       Qt.AlignmentFlag.AlignBottom)

        main_layout.addLayout(bottom_screen_layout)


def run_gui_pilot() -> None:
    PilotApp().run_gui()
