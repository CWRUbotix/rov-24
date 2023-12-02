import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace


def generate_launch_description() -> LaunchDescription:

    gui_path: str = get_package_share_directory('gui')
    teleop_path: str = get_package_share_directory('teleop')
    # Launches Gui
    gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                gui_path, 'launch', 'operator_launch.py'
            )
        ]),
    )

    # Launches flight controls (manual/auto flight)
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                teleop_path, 'launch', 'teleop_launch.py'
            )
        ]),
    )

    namespace_launch = GroupAction(
        actions=[
            PushRosNamespace("surface"),
            gui_launch,
            teleop_launch,
        ]
    )

    return LaunchDescription([
        namespace_launch
    ])
