
from gui.widgets.video_widget import VideoWidget, CameraDescription, CameraType
from PyQt6.QtWidgets import QVBoxLayout, QWidget


class AutoTransplantDebug(QWidget):
    def __init__(self) -> None:
        super().__init__()

        root_layout = QVBoxLayout()
        # TODO could use param probably
        root_layout.addWidget(VideoWidget(CameraDescription(CameraType.SIMULATION,
                                                            "bottom_cam/annotated",
                                                            "Bottom Cam Labeled", *[721, 541])))
        self.setLayout(root_layout)
