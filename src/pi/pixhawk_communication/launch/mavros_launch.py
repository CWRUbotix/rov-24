import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    mavros_path: str = get_package_share_directory('surface_main')

    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                mavros_path, 'launch', 'apm.launch'
            )
        ]),
    )

    return LaunchDescription([
        mavros_launch
    ])
