from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    # Create the launch configuration variables.
    vesc_driver_config = LaunchConfiguration("vesc_driver_config")
    vesc_ackermann_config = LaunchConfiguration("vesc_ackermann_config")
    log_level = LaunchConfiguration("log_level")

    # Define default values.
    default_vesc_driver_config = PathJoinSubstitution(
        [
            FindPackageShare("maila_bringup"),
            "params",
            "vesc_config.yaml",
        ]
    )
    default_vesc_ackermann_config = PathJoinSubstitution(
        [
            FindPackageShare("maila_bringup"),
            "params",
            "vesc_ackermann_config.yaml",
        ]
    )
    default_log_level = "info"

    # Declare arguments commands.
    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value=default_log_level, description="log level"
    )
    declare_vesc_driver_config_cmd = DeclareLaunchArgument(
        "vesc_driver_config",
        default_value=default_vesc_driver_config,
        description="log level",
    )
    declare_vesc_ackermann_config_cmd = DeclareLaunchArgument(
        "vesc_ackermann_config",
        default_value=default_vesc_ackermann_config,
        description="log level",
    )

    # Specify the actions.
    start_vesc_driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("vesc_driver"),
                        "launch",
                        "vesc_driver_node.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "config": vesc_driver_config,
        }.items(),
    )

    start_vesc_ackermann_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("vesc_ackermann"),
                        "launch",
                        "ackermann_to_vesc_node.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "config": vesc_ackermann_config,
            "log_level": log_level,
        }.items(),
    )

    start_vesc_odom_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("vesc_ackermann"),
                        "launch",
                        "vesc_to_odom_node.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "config": vesc_ackermann_config,
            "log_level": log_level,
        }.items(),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_vesc_driver_config_cmd)
    ld.add_action(declare_vesc_ackermann_config_cmd)

    # Add the action to launch the node
    ld.add_action(start_vesc_driver_cmd)
    ld.add_action(start_vesc_odom_cmd)
    ld.add_action(start_vesc_ackermann_cmd)

    return ld
