"""
@file visualize_myactuator_rmd.launch.py
@brief
    Launch file for visualizing MyActuator RMD-X-series URDF
@author
    Tobit Flatscher (github.com/2b-t)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    actuator_parameter_name = 'actuator'
    actuator = LaunchConfiguration(actuator_parameter_name)

    actuator_cmd = DeclareLaunchArgument(
        actuator_parameter_name,
        default_value='X8ProV2',
        description='Type of the actuator'
    )

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('myactuator_rmd_description'),
                    'launch', 'myactuator_rmd.launch.py'
                )
            ]
        ),
        launch_arguments={
            'actuator': actuator,
            'simulation': 'true'
        }.items(),
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz_file = os.path.join(
        get_package_share_directory('myactuator_rmd_description'),
        'rviz',
        'visualize_myactuator_rmd.rviz'
    )
    rviz_node = Node(package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', rviz_file]
    )

    ld = LaunchDescription()
    ld.add_action(actuator_cmd)
    ld.add_action(description_launch)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz_node)
    return ld
