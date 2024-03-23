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
    reader_node: Node = Node(
        namespace='surface',
        package='transceiver',
        executable='serial',
    )

    return LaunchDescription([
        reader_node
    ])
