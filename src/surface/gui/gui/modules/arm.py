
from PyQt5.QtWidgets import QPushButton, QHBoxLayout, QWidget
from gui.event_nodes.publisher import GUIEventPublisher

from px4_msgs.msg import VehicleCommand


class Arm(QWidget):
    """Arm widget for sending Arm Commands."""

    BUTTON_WIDTH = 120
    BUTTON_HEIGHT = 60
    BUTTON_STYLESHEET = 'QPushButton { font-size: 20px; }'

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

        self.arm_publisher: GUIEventPublisher = GUIEventPublisher(
            VehicleCommand,
            "fmu/in/vehicle_command"
        )

    def arm_clicked(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = VehicleCommand.ARMING_ACTION_ARM
        self.arm_publisher.publish(msg)

    def disarm_clicked(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = VehicleCommand.ARMING_ACTION_DISARM
        self.arm_publisher.publish(msg)
