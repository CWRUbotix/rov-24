import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction

from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import SetParameter


def generate_launch_description():

    mavros_path: str = get_package_share_directory('mavros')

    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            os.path.join(
                mavros_path, 'launch', 'apm.launch'
            )
        ])
    )

    # https://github.com/mavlink/mavros/issues/1632
    # Done so RC Override thinks mavros in a gcs.
    system_id_param = SetParameter(name="system_id", value=255)
    component_id_param = SetParameter(name="component_id", value=240)

    mavros_launch_params = GroupAction(
        actions=[
            system_id_param,
            component_id_param,
            mavros_launch
        ]
    )

    return LaunchDescription([
        mavros_launch_params
    ])
