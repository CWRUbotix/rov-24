from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():

    # launches transceiver
    reader_node: Node = Node(
        namespace='surface',
        package='transceiver',
        executable='serial',
    )

    return LaunchDescription([
        reader_node
    ])
