import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    vesc_driver_config = PathJoinSubstitution([
        FindPackageShare('maila_bringup'),
        'params',
        'vesc_config.yaml',
        ]
    )

    vesc_ackermann_config = PathJoinSubstitution([
        FindPackageShare('maila_bringup'),
        'params',
        'vesc_ackermann_config.yaml',
        ]
    )

    start_vesc_driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
            PathJoinSubstitution(
                [
                    FindPackageShare('maila_bringup'),
                    "launch",
                    "vesc_ackermann_node.launch.py",
                ]
            )
            ]
        ),
        launch_arguments={
            "config": vesc_ackermann_config,
        }.items()
    )

    start_vesc_ackermann_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
            PathJoinSubstitution(
                [
                    FindPackageShare('vesc_driver'),
                    "launch",
                    "vesc_driver_node.launch.py",
                ]
            )
            ]
        ),
        launch_arguments={
            "config": vesc_driver_config,
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(start_vesc_driver_cmd)
    ld.add_action(start_vesc_ackermann_cmd)

    return ld
