import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:

    surface_path: str = get_package_share_directory('surface_main')
    gui_path: str = get_package_share_directory('gui')

    all_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                surface_path, 'launch', 'surface_all_nodes_launch.py'
            )
        ]),
    )

    # Launches livestream gui
    gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                gui_path, 'launch', 'pilot_launch.py'
            )
        ]),
        launch_arguments=[('gui', 'livestream')]
    )

    return LaunchDescription([
        all_launch,
        gui_launch
    ])
