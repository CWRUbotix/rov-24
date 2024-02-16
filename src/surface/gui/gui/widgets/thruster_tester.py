import os
import time
from threading import Thread

from ament_index_python.packages import get_package_share_directory
from gui.event_nodes.client import GUIEventClient
from gui.event_nodes.subscriber import GUIEventSubscriber
from mavros_msgs.srv import CommandLong, ParamPull
from PyQt6.QtCore import pyqtSignal, pyqtSlot
from PyQt6.QtGui import QIntValidator, QPixmap
from PyQt6.QtWidgets import (QGridLayout, QHBoxLayout, QLabel, QLineEdit,
                             QPushButton, QVBoxLayout, QWidget)
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter

from rov_msgs.msg import VehicleState


class ThrusterTester(QWidget):
    """Widget to command the pixhawk to test the thrusters, and reassign thruster ports."""

    TEST_LENGTH: float = 2.0  # time between adjacent tests of individual thrusters
    TEST_THROTTLE: float = 0.50  # 50%
    MOTOR_COUNT = 8

    test_command_callback_signal = pyqtSignal(CommandLong.Response)
    vehicle_state_callback_signal = pyqtSignal(VehicleState)
    pull_motors_callback_signal = pyqtSignal(ParamPull.Response)
    param_send_callback_signal = pyqtSignal(SetParameters.Response)

    def __init__(self) -> None:
        super().__init__()

        # TODO Our ROS Node count is getting kind of crazy lol
        self.test_cmd_client = GUIEventClient(
            CommandLong,
            "mavros/cmd/command",
            self.test_command_callback_signal
        )
        self.test_command_callback_signal.connect(self.command_response_handler)

        self.vehicle_state_subscriber = GUIEventSubscriber(
            VehicleState,
            "vehicle_state_event",
            self.vehicle_state_callback_signal
        )
        self.vehicle_state_callback_signal.connect(self.vehicle_state_callback)

        self.param_pull_client = GUIEventClient(
            ParamPull,
            "mavros/param/pull",
            self.pull_motors_callback_signal
        )
        self.pull_motors_callback_signal.connect(self.pull_param_handler)

        self.param_send_client = GUIEventClient(
            SetParameters,
            "mavros/param/set_parameters",
            self.param_send_callback_signal
        )
        self.param_send_callback_signal.connect(self.param_send_signal_handler)

        layout = QVBoxLayout()

        heading = QLabel("Thruster Pin Configuration")

        pin_numbers_grid = QGridLayout()
        self.pin_input_widgets: list[QLineEdit] = []

        pin_number_validator = QIntValidator()
        pin_number_validator.setRange(1, self.MOTOR_COUNT)

        for i in range(1, self.MOTOR_COUNT + 1):
            pin_label = QLabel(str(i))
            pin_label.setMaximumWidth(20)
            pin_input = QLineEdit()
            pin_input.setValidator(pin_number_validator)
            pin_input.insert(str(i))
            pin_input.setMaximumWidth(20)
            self.pin_input_widgets.append(pin_input)

            on_first_column = i - 1 < self.MOTOR_COUNT / 2
            column = 1 if on_first_column else 4
            row = i if on_first_column else i - self.MOTOR_COUNT // 2
            pin_numbers_grid.addWidget(pin_label, row, column)
            pin_numbers_grid.addWidget(pin_input, row, column + 1)

        pin_numbers_grid.setColumnStretch(0, 1)
        pin_numbers_grid.setColumnStretch(3, 1)
        pin_numbers_grid.setColumnStretch(6, 1)

        pin_assignment_button = QPushButton()
        pin_assignment_button.setText("Send Pin Assignments")
        pin_assignment_button.clicked.connect(self.send_pin_assignments)

        test_button = QPushButton()
        test_button.setText("Test Thrusters")
        test_button.clicked.connect(self.async_send_test_message)

        gui_path = get_package_share_directory('gui')
        picture_path = os.path.join(gui_path, 'doc', 'images', 'vectored6dof-frame.png')
        image = QLabel()
        pixmap = QPixmap(picture_path)
        image.setPixmap(pixmap.scaledToHeight(200))

        layout.addWidget(heading)
        layout.addLayout(pin_numbers_grid)
        layout.addWidget(pin_assignment_button)
        layout.addWidget(test_button)

        main_layout = QHBoxLayout()
        main_layout.addWidget(image)
        main_layout.addLayout(layout)

        self.setLayout(main_layout)

    def test_motor_for_time(self, motor_index: int, throttle: float, duration: float) -> None:
        """
        Run a motor for an (approximate) length of time (blocking).

        Based on https://github.com/mavlink/qgroundcontrol/blob/f79b466/src/Vehicle/Vehicle.cc

        Parameters
        ----------
        motor_index : int
            A motor index, from 1 to 8
        throttle : float
            A float from -1 to 1, where -1 is full reverse and 1 is full forward
        duration : float
            Time in seconds to run the motor

        """
        throttle = max(min(throttle, 1.0), -1.0)  # Clamp the throttle between -1 and 1
        throttle_percent = 50 * throttle + 50  # Rescale to be from 0 to 100, with 50 at the center

        start_time = time.time()
        while time.time() - start_time < duration:
            self.test_cmd_client.send_request_async(
                CommandLong.Request(
                    command=209,  # MAV_CMD_DO_MOTOR_TEST
                    param1=float(motor_index),  # Motor number
                    param2=0.0,  # MOTOR_TEST_THROTTLE_PERCENT
                    param3=float(throttle_percent),
                    param4=0.0,  # Time between tests
                    param5=0.0,  # Number of motors to test
                    param6=2.0  # MOTOR_TEST_ORDER_BOARD
                )
            )

            time.sleep(0.05)

    def async_send_test_message(self) -> None:
        """Asynchronously send a test message."""
        Thread(target=self.send_test_message, daemon=True, name="thruster_test_thread").start()

    def send_test_message(self) -> None:
        """Send a test message for each motor."""
        for motor_index in range(self.MOTOR_COUNT):
            self.test_cmd_client.get_logger().info(f"Testing thruster {motor_index + 1}")
            self.test_motor_for_time(motor_index, self.TEST_THROTTLE, self.TEST_LENGTH)
            self.test_motor_for_time(motor_index, 0.0, 0.5)

    @pyqtSlot(CommandLong.Response)
    def command_response_handler(self, res: CommandLong.Response) -> None:
        """
        Log response to CommandLong service.

        Parameters
        ----------
        res : CommandLong.Response
            CommandLong.Response message.

        """
        self.test_cmd_client.get_logger().info(f"Test response: {res.success}, {res.result}")

    def vehicle_state_callback(self, msg: VehicleState) -> None:
        """
        Pull params after the pixhawk is connected.

        Parameters
        ----------
        msg : VehicleState
            VehicleState message.

        """
        if msg.pixhawk_connected:
            self.pull_param()

    def pull_param(self) -> None:
        """Pull params off pixhawk."""
        self.param_pull_client.send_request_async(ParamPull.Request())

    @pyqtSlot(ParamPull.Response)
    def pull_param_handler(self, res: ParamPull.Response) -> None:
        """
        Log response to ParamPull service.

        Parameters
        ----------
        res : ParamPull.Response
            ParamPull.Response message.

        """
        self.param_pull_client.get_logger().info((f"Success: {res.success},"
                                                  f"param_received: {res.param_received}."))
        # TODO should this free itself after success?

    def send_pin_assignments(self) -> None:
        """Send pin assignments to the pixhawk."""
        # https://wiki.ros.org/mavros/Plugins#param
        # https://ardupilot.org/copter/docs/parameters.html#servo10-parameters

        pin_input_list = [int(x.text()) for x in self.pin_input_widgets]

        # Data validation to check that all motor values are unique
        if len(pin_input_list) == len(set(pin_input_list)):
            param_list = []
            for i, pin_input in enumerate(pin_input_list):
                # https://ardupilot.org/copter/docs/parameters.html#servo1-function-servo-output-function
                param = Parameter(f"SERVO{i + 1}_FUNCTION",
                                  Parameter.Type.INTEGER,
                                  pin_input + 32).to_parameter_msg()
                param_list.append(param)

            req = SetParameters.Request(parameters=param_list)
            self.param_send_client.send_request_async(req)
        else:
            self.param_send_client.get_logger().error("Duplicate Motor Output ignoring request.")

    @pyqtSlot(SetParameters.Response)
    def param_send_signal_handler(self, res: SetParameters.Response) -> None:
        """
        Log response to SetParameters service.

        Parameters
        ----------
        res : SetParameters.Response
            SetParameters.Response message.

        """
        self.param_send_client.get_logger().info(f"Parameter setting success: {res.results}.")
