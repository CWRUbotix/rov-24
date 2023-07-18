
from PyQt5.QtWidgets import QPushButton, QHBoxLayout, QWidget
from PyQt5.QtCore import pyqtSignal, pyqtSlot

from gui.event_nodes.client import GUIEventClient

from mavros_msgs.srv import CommandBool


class Arm(QWidget):
    """Arm widget for sending Arm Commands."""

    ARMING_ACTION_ARM = True
    ARMING_ACTION_DISARM = False
    BUTTON_WIDTH = 120
    BUTTON_HEIGHT = 60
    BUTTON_STYLESHEET = 'QPushButton { font-size: 20px; }'

    signal: pyqtSignal = pyqtSignal(CommandBool.Response)

    def __init__(self):

        super().__init__()

        layout: QHBoxLayout = QHBoxLayout()
        self.setLayout(layout)

        arm_button = QPushButton()
        disarm_button = QPushButton()

        arm_button.setText("Arm")
        disarm_button.setText("Disarm")

        arm_button.setMinimumWidth(self.BUTTON_WIDTH)
        disarm_button.setMinimumWidth(self.BUTTON_WIDTH)

        arm_button.setMinimumHeight(self.BUTTON_HEIGHT)
        disarm_button.setMinimumHeight(self.BUTTON_HEIGHT)

        arm_button.setStyleSheet(self.BUTTON_STYLESHEET)
        disarm_button.setStyleSheet(self.BUTTON_STYLESHEET)

        arm_button.clicked.connect(self.arm_clicked)
        disarm_button.clicked.connect(self.disarm_clicked)

        layout.addWidget(arm_button)
        layout.addWidget(disarm_button)

        self.signal.connect(self.arm_status)

        self.arm_client: GUIEventClient = GUIEventClient(
            CommandBool,
            "/mavros/cmd/arming",
            self.signal
        )

    def arm_clicked(self):
        self.client_handler(self.ARMING_ACTION_ARM)

    def disarm_clicked(self):
        self.client_handler(self.ARMING_ACTION_DISARM)

    def client_handler(self, arming: bool):
        srv_req = CommandBool.Request()
        srv_req.value = arming
        self.arm_client.send_request_async(srv_req)

    @pyqtSlot(CommandBool.Response)
    def arm_status(self, res: CommandBool.Response):
        # TODO? could check against /mavros/state for confirmation
        if res:
            self.arm_client.get_logger().info("Has been armed or disarmed.")
        else:
            self.arm_client.get_logger().error("Has not armed or disarmed")
