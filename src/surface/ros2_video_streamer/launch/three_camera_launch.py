import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    Launch 3 video streamers with cam names `front_cam`, `manip_cam`, `bottom_cam`.

    You'll need `bottom_cam.mp4`, `front_cam.mp4`, & `manip_cam.mp4` in `ros2_video_streamer`.
    """
    front_cam_launcher:  IncludeLaunchDescription = create_cam_launcher('front')
    manip_cam_launcher:  IncludeLaunchDescription = create_cam_launcher('manip')
    bottom_cam_launcher: IncludeLaunchDescription = create_cam_launcher('bottom')

    return launch.LaunchDescription([
        manip_cam_launcher,
        front_cam_launcher,
        bottom_cam_launcher
    ])


def create_cam_launcher(camera_name: str) -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros2_video_streamer'),
                'launch',
                'ros2_video_streamer_node_launch.py'
            )
        ]),
        # Need to set normally camera_name-derived args manually b/c ROS gets
        # confused and makes them the same when we launch simultaneously
        launch_arguments=[
            ('type', 'video'),
            ('camera_name', f'{camera_name}_cam'),
            ('image_topic_name', f'/{camera_name}_cam/image_raw'),
            ('info_topic_name', f'/{camera_name}_cam/camera_info'),
            ('file_name', f'{camera_name}_cam.mp4')
        ]
    )
