import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vesc_config = PathJoinSubstitution([
        FindPackageShare('maila_bringup'),
        'params',
        'vesc_config.yaml',
        ]
    )
    
    found_package = [
                        FindPackageShare('vesc_driver'),
                        "launch",
                        "vesc_driver_node.launch.py"
                    ]
    print(found_package)
    
    return LaunchDescription(
        [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                PathJoinSubstitution(
                    found_package
                )
                ]
            ),
            launch_arguments={
                "config": vesc_config,
            }.items()
        )
        
        ]
    )