import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

# from launch.substitutions import Command

NS = "simulation"


def generate_launch_description():
    rov_gazebo_path: str = get_package_share_directory("rov_gazebo")
    # ros_gz_sim_path: str = get_package_share_directory("ros_gz_sim")
    surface_main_path: str = get_package_share_directory("surface_main")

    world_file = "bluerov2_heavy_underwater.world"

    world_path: str = os.path.join(rov_gazebo_path, "worlds", world_file)
    # Process the URDF file
    # xacro_file = os.path.join(rov_gazebo_path, "description", "rov.xacro")
    # robot_description = Command(["xacro ", xacro_file])
    # params = {"robot_description": robot_description}

    # pool_file = os.path.join(rov_gazebo_path, "description", "pool.xacro")
    # pool_description = Command(["xacro ", pool_file])
    # pool_params = {"robot_description": pool_description}

    # # Create a robot_state_publisher node
    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="screen",
    #     parameters=[params],
    #     namespace=NS,
    #     emulate_tty=True
    # )

    # pool_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="screen",
    #     parameters=[pool_params],
    #     namespace=NS,
    #     remappings=[(f"/{NS}/robot_description", f"/{NS}/pool_description")],
    #     emulate_tty=True
    # )

    # # Launches Gazebo
    # gazeboLaunch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [os.path.join(ros_gz_sim_path, "launch", "gz_sim.launch.py")]
    #     ),
    #     launch_arguments={"gz_args": world_path}.items(),
    # )

    # TODO gz_sim launch might be nice
    start_gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '3', '-r', world_path],
        output='screen'
    )

    start_ardusub = ExecuteProcess(
        cmd=["ardusub", '-S', '-w', '-M', 'JSON', '-I0'],
        output='screen'
    )

    # Translate messages MAV <-> ROS
    mav_ros_node = Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            namespace='mavros',
            parameters=[
                {"system_id": 255},
                # TODO check if needed
                {"component_id": 240},
                {"fcu_url": "tcp://localhost"},
                {"gcs_url": "udp://@localhost:14550"},
                {"plugin_allowlist": ["rc_io", "sys_status", "command"]}
            ],
            remappings=[
                # (f'/{NS}/mavros/setpoint_velocity/cmd_vel',
                #          '/mavros/setpoint_velocity/cmd_vel'),
                        (f'/{NS}/mavros/cmd/arming', '/mavros/cmd/arming'),
                        (f'/{NS}/mavros/rc/override', '/mavros/rc/override')]
    )

    # # Spawn entity
    # gz_spawn_entity = Node(
    #     package="ros_gz_sim",
    #     executable="create",
    #     output="screen",
    #     arguments=[
    #         "-topic",
    #         "robot_description",
    #         "-name",
    #         "ROV",
    #         "-allow_renaming",
    #         "true",
    #     ],
    #     namespace=NS,
    # )

    # gz_spawn_pool = Node(
    #     package="ros_gz_sim",
    #     executable="create",
    #     output="screen",
    #     arguments=[
    #         "-topic",
    #         "pool_description",
    #         "-name",
    #         "pool",
    #         "-allow_renaming",
    #         "true",
    #     ],
    #     namespace=NS,
    # )

    # Not using keyboard launch file
    # TODO?
    # I think we should probably switch over all our single
    # Node launch files to something like this
    keyboard_driver = Node(
        package="keyboard_driver",
        executable="keyboard_driver_node",
        output="screen",
        remappings=[(f'/{NS}/mavros/setpoint_velocity/cmd_vel',
                     '/mavros/setpoint_velocity/cmd_vel')],
        emulate_tty=True
    )

    # cam_bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     namespace=NS,
    #     name="cam_bridge",
    #     arguments=[
    #         "/bottom_cam/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
    #         "/bottom_cam/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
    #         "/front_cam/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
    #         "/front_cam/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
    #         "/manip_cam/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
    #         "/depth_cam@sensor_msgs/msg/Image@gz.msgs.Image",
    #         "/depth_cam/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
    #     ],
    #     remappings=[
    #         (f"/{NS}/bottom_cam/image_raw", "/bottom_cam/image_raw"),
    #         (f"/{NS}/bottom_cam/camera_info", "/bottom_cam/camera_info"),
    #         (f"/{NS}/front_cam/image_raw", "/front_cam/image_raw"),
    #         (f"/{NS}/front_cam/camera_info", "/front_cam/camera_info"),
    #         (f"/{NS}/manip_cam/image_raw", "/manip_cam/image_raw"),
    #         (f"/{NS}/depth_cam", "/depth_cam/image_raw"),
    #         (f"/{NS}/depth_cam/points", "/depth_cam/points"),
    #     ],
    #     output="screen",
    #     emulate_tty=True
    # )

    # Launches Surface Nodes
    surface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(surface_main_path, "launch", "surface_all_nodes_launch.py")]
        ),
    )

    namespace_launch = GroupAction(
        actions=[
            PushRosNamespace(NS),
            mav_ros_node,
            keyboard_driver,
        ]
    )

    return LaunchDescription(
        [
            # robot_state_publisher,
            # pool_state_publisher,
            # gazeboLaunch,
            start_gazebo,
            start_ardusub,
            namespace_launch,
            # gz_spawn_entity,
            # gz_spawn_pool,
            # thrust_bridge,
            # cam_bridge,
            surface_launch,
        ]
    )
