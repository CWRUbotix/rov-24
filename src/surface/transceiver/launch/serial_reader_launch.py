from launch_ros.actions import Node
from launch.launch_description import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    """
    Generate LaunchDescription for transceiver.

    Returns
    -------
    LaunchDescription
        Launches serial_reader node.

    """
    # launches transceiver
    reader_node = Node(
        package='transceiver',
        executable='serial',
        emulate_tty=True,
        output="screen",
        remappings=[("/surface/gui/transceiver_data", "/surface/transceiver_data"),
                    ("/surface/gui/float_command", "/surface/float_command"),
                    ("/surface/gui/float_serial", "/surface/float_serial")]
    )

    return LaunchDescription([
        reader_node
    ])
