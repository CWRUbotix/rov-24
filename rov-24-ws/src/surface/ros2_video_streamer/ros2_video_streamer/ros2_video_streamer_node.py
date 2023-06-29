import os
import cv2
from cv2 import VideoCapture, Mat
# import yaml
import rclpy

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from builtin_interfaces.msg import Time
from ament_index_python.packages import get_package_share_directory


class VideoStreamerNode(Node):
    """ROS Camera simulator Node; reads video file & pubs ROS Images."""

    def __init__(self):
        super().__init__('ros2_video_streamer_temp_name',
                         parameter_overrides=[])

        self.load_launch_parameters()

        # config = self.load_config_file(self.config_file_path)
        # if config is not None:
        #     self.camera_info = self.get_camera_info(config)
        # else:
        #     self.camera_info = None

        self.bridge = CvBridge()

        # Publishers
        self.image_publisher_ = self.create_publisher(
            Image,
            self.image_topic_name,
            5)
        self.camera_info_publisher_ = self.create_publisher(
            CameraInfo,
            self.info_topic_name,
            1)

        if not os.path.isfile(self.path):
            raise RuntimeError(f'Invalid video path: {self.path}')

        if self.type == 'video':
            self.vc: VideoCapture = cv2.VideoCapture(self.path)
            self.vc.set(cv2.CAP_PROP_POS_MSEC, self.start)
            video_fps: float = self.vc.get(cv2.CAP_PROP_FPS)
        elif self.type == 'image':
            self.image = cv2.imread(self.path)
            video_fps = 10
        else:
            raise ValueError(f'Unknown type: {self.type}')

        self.timer = self.create_timer(1.0/video_fps, self.image_callback)
        self.get_logger().info(f'Publishing image at {video_fps} fps')

    def load_launch_parameters(self):
        """Load the launch ROS parameters."""
        self.declare_parameter('image_topic_name',
                               value='/simulated_cam/image_raw')
        self.declare_parameter('info_topic_name',
                               value='/simulated_cam/camera_info')
        self.declare_parameter('file_name', value='simulated_cam.mp4')
        # self.declare_parameter('config_file_path', value='')
        self.declare_parameter('loop', value=True)
        self.declare_parameter('frame_id', value='')
        self.declare_parameter('type', value='')
        self.declare_parameter('start', value=0)

        self.image_topic_name = self.get_parameter('image_topic_name')\
            .get_parameter_value().string_value
        self.info_topic_name = self.get_parameter('info_topic_name')\
            .get_parameter_value().string_value
        self.file_name = self.get_parameter('file_name')\
            .get_parameter_value().string_value
        # self.config_file_path = self.get_parameter('config_file_path')\
        #     .get_parameter_value().string_value
        self.loop = self.get_parameter('loop')\
            .get_parameter_value().bool_value
        self.frame_id_ = self.get_parameter('frame_id')\
            .get_parameter_value().string_value
        self.type = self.get_parameter('type')\
            .get_parameter_value().string_value
        self.start = self.get_parameter('start')\
            .get_parameter_value().integer_value

        self.path = os.path.join(get_package_share_directory(
                                 'ros2_video_streamer'), self.file_name)

    # def load_config_file(self, file_path: str):
    #     """Attempt to load the optional config yaml file."""
    #     try:
    #         path = os.path.join(
    #             get_package_share_directory('ros2_video_streamer'),
    #             'ros2_video_streamer', 'config', file_path)
    #         f = open(path)
    #         return yaml.safe_load(f)
    #     except IOError:
    #         self.get_logger().warning(
    #             'Could not find calibration file ' + file_path +
    #             ', will proceed without a calibration file')
    #         return None

    # def get_camera_info(self, config):
    #     """Extract camera info from the provided config file."""
    #     ci = CameraInfo()
    #     ci.header.frame_id = self.frame_id_
    #     ci.width = config['image_width']
    #     ci.height = config['image_height']
    #     ci.distortion_model = config['distortion_model']
    #     ci.d = list(float(v) for v in config['distortion_coefficients']['data'])
    #     ci.k = list(float(v) for v in config['camera_matrix']['data'])
    #     ci.r = list(float(v) for v in config['rectification_matrix']['data'])
    #     ci.p = list(float(v) for v in config['projection_matrix']['data'])
    #     return ci

    def image_callback(self):
        """Process an image or frame of video."""
        if self.type == 'video':
            rval, image = self.vc.read()
            rval: bool = rval
            image: Mat = image
            if not rval and not self.loop:
                self.get_logger().info('End of video, closing node...')
                self.timer.cancel()
                self.destroy_node()
                exit()
            elif not rval and self.loop:
                self.vc.set(cv2.CAP_PROP_POS_MSEC, 0)
                rval, image = self.vc.read()
        elif self.type == 'image':
            image = self.image
        else:
            raise ValueError(f'Unknown type: {self.type}')

        time_msg = self.get_clock().now().to_msg()
        img_msg = self.get_image_msg(image, time_msg)

        # if self.camera_info is not None:
        #     self.camera_info.header.stamp = time_msg
        #     self.camera_info_publisher_.publish(self.camera_info)

        self.image_publisher_.publish(img_msg)

    def get_image_msg(self, image: Mat, time: Time) -> Image:
        """
        Convert cv2 image to ROS2 Image with CvBridge cv2 -> image msg.

        :param image: cv2 image
        :return: sensor_msgs/Imag
        """
        inverted_image: Mat = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        img_msg: Image = self.bridge.cv2_to_imgmsg(inverted_image)
        img_msg.header.stamp = time
        return img_msg


def main():
    rclpy.init()
    video_streamer_node = VideoStreamerNode()
    rclpy.spin(video_streamer_node)
    video_streamer_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
