from enum import IntEnum
from collections import deque

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from cv_bridge import CvBridge
from cv2.typing import MatLike

from sensor_msgs.msg import Image

from flight_control.image_processing.square_detector import SquareDetector

from rov_msgs.msg import PixhawkInstruction, Manip
from rov_msgs.srv import AutonomousFlight


DROP_FRAMES = 2
QUEUE_DECAY_RATE_FRAMES = 5
QUEUE_MINIMUM_HITS = 1
SCANNING_SPEED = 0.05
APPROACHING_SPEED = 0.05

RELEASE_THRESHOLD = 0.2

IntCoordinate = tuple[int, int]
FloatCoordinate = tuple[float, float]


class Stage(IntEnum):
    SCANNING = 0
    APPROACHING = 1
    RELEASING = 2


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

        self.manip_publisher = self.create_publisher(
            Manip,
            'manipulator_control',
            QoSPresetProfiles.DEFAULT.value
        )

        self.cv_bridge: CvBridge = CvBridge()

        self.frame_index = 0
        self.last_processed_frame = 0
        self.last_decay = 0
        self.hit_queue: deque[list[IntCoordinate]] = deque()
        self.stage = Stage.SCANNING

    def task_control_callback(self, request: AutonomousFlight.Request,
                              response: AutonomousFlight.Response) -> AutonomousFlight.Response:
        self.current_state = request.state
        response.current_state = request.state
        return response

    def is_target_visible(self) -> bool:
        result = len(self.hit_queue) >= QUEUE_MINIMUM_HITS
        print(f'Target is {"NOT" if not result else ""} visible ({len(self.hit_queue)} >= {QUEUE_MINIMUM_HITS})')
        return result

    def last_target_corners_pos(self) -> list[IntCoordinate]:
        position: list[IntCoordinate] = []

        # Get the average of the points in position
        for points in self.hit_queue:
            if len(position) == 0:
                position = points
            else:
                # Ignore everything except the best guess (first 4 points)
                for i, point in enumerate(points[:4]):
                    position[i] = (
                        position[i][0] + point[0],
                        position[i][1] + point[1]
                    )

        for i, point in enumerate(position):
            position[i] = (
                int(point[0] / len(self.hit_queue)),
                int(point[1] / len(self.hit_queue))
            )

        return position

    def last_target_center_pos(self) -> IntCoordinate:
        position: IntCoordinate = (0, 0)

        corners = self.last_target_corners_pos()

        # Average the corners
        for corner in corners:
            position = (
                position[0] + corner[0],
                position[1] + corner[1]
            )

        position = (
            int(position[0] / len(corners)),
            int(position[1] / len(corners))
        )

        return position


    def handle_frame(self, frame: Image) -> None:
        if self.current_state != AutonomousFlight.Request.START:
            return

        print('Height:', frame.height, 'Width:', frame.width)

        self.frame_index += 1

        if self.is_target_visible():
            relative_corners_pos: list[FloatCoordinate] = []
            for point in self.last_target_corners_pos():
                relative_corners_pos.append((point[0] / frame.height, point[1] / frame.width))

            center_pos = self.last_target_center_pos()
            relative_center_pos: FloatCoordinate = (
                center_pos[0] / frame.height,
                center_pos[1] / frame.width
            )

            print(f'Target is at: {relative_center_pos} with corners: {relative_corners_pos}')

        if self.stage == Stage.SCANNING:
            if self.is_target_visible():
                print('Found target, APPROACHING')
                self.stage = Stage.APPROACHING
            else:
                command = PixhawkInstruction(
                    vertical=SCANNING_SPEED,
                    author=PixhawkInstruction.AUTONOMOUS_CONTROL
                )
                self.pixhawk_control.publish(command)

        elif self.stage == Stage.APPROACHING:
            if self.is_target_visible():
                # Release if we're very close to the target
                target_is_huge = True
                for corner in relative_corners_pos:
                    for dim in corner:
                        if RELEASE_THRESHOLD < dim < 1.0 - RELEASE_THRESHOLD:
                            target_is_huge = False
                if target_is_huge:
                    self.stage = Stage.RELEASING
                    return

                print(f'Right: {0.5 - relative_center_pos[1]} | Left: {0.5 - relative_center_pos[0]}')

                command = PixhawkInstruction(
                    vertical=-1 * APPROACHING_SPEED,
                    roll=(0.5 - relative_center_pos[1]) * 0.2,
                    pitch=(0.5 - relative_center_pos[0]) * 0.2,
                    author=PixhawkInstruction.AUTONOMOUS_CONTROL
                )
                self.pixhawk_control.publish(command)
            else:
                print('Lost target, SCANNING')
                self.stage = Stage.SCANNING

        elif self.stage == Stage.RELEASING:
            command = PixhawkInstruction(
                vertical=0.0,
                roll=0.0,
                pitch=0.0,
                author=PixhawkInstruction.AUTONOMOUS_CONTROL
            )
            self.pixhawk_control.publish(command)

            manip_msg = Manip(manip_id="right", activated=False)
            self.manip_publisher.publish(manip_msg)

        print('AUTO TRANSPLANT RECEIVED FRAME: ', end='')
        if self.frame_index > self.last_processed_frame + DROP_FRAMES:
            print('PROCESSING')

            self.last_processed_frame = self.frame_index

            cv_image: MatLike = self.cv_bridge.imgmsg_to_cv2(
                frame, desired_encoding='passthrough')

            square_detector = SquareDetector(False)

            corners, result_img = \
                square_detector.process_image(cv_image, True, False, False)
            print('Corners: ', corners)

            print(f'Appending? {corners is not None} and {corners is not None and len(corners) > 4}')

            if corners is not None and len(corners) >= 4:
                self.hit_queue.append(corners)
                print(f'Appended! {len(self.hit_queue)} {self.hit_queue}')

            if self.frame_index > self.last_decay + QUEUE_DECAY_RATE_FRAMES:
                self.last_decay = self.frame_index
                if len(self.hit_queue) > 0:
                    self.hit_queue.pop()

            if result_img is not None:
                print(result_img.shape)

                annotated_frame: Image = self.cv_bridge.cv2_to_imgmsg(result_img, encoding='passthrough')

                self.annotated_bottom_pub.publish(annotated_frame)
        else:
            print(f'DISCARDING ({self.frame_index - self.last_processed_frame} / {DROP_FRAMES})')


def main() -> None:
    rclpy.init()
    auto_docker = AutoDocker()
    executor = MultiThreadedExecutor()
    rclpy.spin(auto_docker, executor=executor)
