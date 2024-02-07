import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace


def generate_launch_description() -> LaunchDescription:

    gui_path: str = get_package_share_directory("gui")
    flight_control_path: str = get_package_share_directory("flight_control")
    vehicle_manager_path: str = get_package_share_directory("vehicle_manager")

    # Launches GUI
    gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(gui_path, "launch", "operator_launch.py")]
        ),
    )

    # Launches flight_control (auto docking, manual control, etc.)
    flight_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(flight_control_path, "launch", "flight_control_launch.py")]
        ),
    )

    # Launches Vehicle Manager
    vehicle_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(vehicle_manager_path, "launch", "vehicle_manager_launch.py")]
        ),
    )

    namespace_launch = GroupAction(
        actions=[
            PushRosNamespace("surface"),
            gui_launch,
            flight_control_launch,
            vehicle_manager_launch,
        ]
    )

    return LaunchDescription([namespace_launch])
