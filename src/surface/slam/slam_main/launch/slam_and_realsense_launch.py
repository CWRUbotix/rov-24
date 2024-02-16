import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:

    slam_path: str = get_package_share_directory('slam_main')
    realsense_path: str = get_package_share_directory('realsense')

    rtabmap_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                slam_path, 'launch', 'rtabmap_launch.py'
            )
        ]),
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                realsense_path, 'launch', 'realsense_launch.py'
            )
        ]),
    )

    return LaunchDescription([
        rtabmap_slam_launch,
        realsense_launch,
    ])
