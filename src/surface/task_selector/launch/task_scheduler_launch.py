from launch.launch_description import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    # launches main task scheduler
    selector_node: Node = Node(
        package='task_selector',
        executable='selector',
        emulate_tty=True,
        output='screen'
    )

    # Manual Control
    manual_control_node: Node = Node(
        package='task_selector',
        executable='manual_control_node',
        remappings=[('/surface/manipulator_control', '/tether/manipulator_control'),
                    ('/surface/mavros/rc/override', '/tether/mavros/rc/override')],
        emulate_tty=True,
        output='screen'
    )

    # # example of node requesting tasks
    # ex_request_client_node: Node = Node(
    #     package='task_selector',
    #     executable='ex_request_client'
    # )

    # # example task- run a 10 second timer
    # ex_timed_task_node: Node = Node(
    #     package='task_selector',
    #     executable='ex_timed_task'
    # )

    # # example task- say the task is finished
    # ex_basic_task_node: Node = Node(
    #     package='task_selector',
    #     executable='ex_basic_task'
    # )

    # # example task- say good morning
    # ex_morning_task_node: Node = Node(
    #     package='task_selector',
    #     executable='ex_morning_task'
    # )

    return LaunchDescription([
        selector_node,
        manual_control_node,
        # ex_request_client_node,
        # ex_timed_task_node,
        # ex_basic_task_node,
        # ex_morning_task_node
    ])
