#!/usr/bin/env python3
"""
Author: Tobit Flatscher (github.com/2b-t)

Launch file for loading Gazebo simulation of MyActuator RMD-X-series
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
    controller_parameter_name = 'controller'
    controller = LaunchConfiguration(controller_parameter_name)
    # Add arguments for starting the different GUIs
    # Add argument for controller file

    actuator_cmd = DeclareLaunchArgument(
        actuator_parameter_name,
        choices=['X8ProV2'],
        default_value='X8ProV2',
        description='Type of the actuator'
    )
    controller_cmd = DeclareLaunchArgument(
        controller_parameter_name,
        choices=['forward_position_controller', 'joint_trajectory_controller'],
        default_value='joint_trajectory_controller',
        description='Controller to be spawned'
    )

    description = IncludeLaunchDescription(
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
        }.items()
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[controller, '-c', '/controller_manager']
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch', 'gazebo.launch.py'
                )
            ]
        )
    )
    gazebo_spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='gazebo_spawner',
        arguments=['-entity', actuator, '-topic', 'robot_description'],
        output="screen"
    )

    rqt_controller_manager = Node(
        package='rqt_controller_manager',
        executable='rqt_controller_manager',
        name='rqt_controller_manager',
        output="screen"
    )
    rqt_joint_trajectory_controller = Node(
        package='rqt_joint_trajectory_controller',
        executable='rqt_joint_trajectory_controller',
        name='rqt_joint_trajectory_controller',
        output="screen"
    )

    rviz_file = os.path.join(
        get_package_share_directory('myactuator_rmd_description'),
        'rviz',
        'visualize_myactuator_rmd.rviz'
    )
    rviz = Node(package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', rviz_file]
    )

    ld = LaunchDescription()
    ld.add_action(actuator_cmd)
    ld.add_action(controller_cmd)
    ld.add_action(description)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(controller_spawner)
    ld.add_action(gazebo)
    ld.add_action(gazebo_spawn_robot)
    ld.add_action(rqt_controller_manager)
    ld.add_action(rqt_joint_trajectory_controller)
    ld.add_action(rviz)
    return ld
