import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from sensor_msgs.msg import Image

from rov_msgs.msg import PixhawkInstruction
from rov_msgs.srv import AutonomousFlight


class AutoDocker(Node):

    def __init__(self) -> None:
        super().__init__('auto_docker')

        self.control_server = self.create_service(
            AutonomousFlight, 'auto_control_toggle', self.task_control_callback)

        self.pixhawk_control = self.create_publisher(
            PixhawkInstruction,
            "pixhawk_control",
            QoSPresetProfiles.DEFAULT.value
        )

        self.current_state = AutonomousFlight.Request.STOP

        # self.front_cam_subscriber = self.create_subscription(
        #     Image,
        #     'front_cam/image_raw',
        #     lambda frame: self.handle_frame(frame, 'bottom'),
        #     QoSPresetProfiles.DEFAULT.value
        # )

        self.create_subscription(
            Image,
            'bottom_cam/image_raw',
            self.handle_frame,
            # lambda frame: self.handle_frame(frame, 'front'),
            QoSPresetProfiles.DEFAULT.value
        )

        self.annotated_bottom_pub = self.create_publisher(
            Image,
            'bottom_cam/annotated',
            QoSPresetProfiles.DEFAULT.value
        )

    def task_control_callback(self, request: AutonomousFlight.Request,
                              response: AutonomousFlight.Response) -> AutonomousFlight.Response:
        self.current_state = request.state
        response.current_state = request.state
        return response

    def handle_frame(self, frame: Image) -> None:
        print('============================= AUTO TRANSPLANT: RECEIVED FRAME =============================')
        self.annotated_bottom_pub.publish(frame)


def main() -> None:
    rclpy.init()
    auto_docker = AutoDocker()
    executor = MultiThreadedExecutor()
    rclpy.spin(auto_docker, executor=executor)
