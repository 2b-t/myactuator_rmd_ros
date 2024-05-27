"""
@file myactuator_rmd_simulation_control.launch.py
@brief
    Launch file for loading Gazebo simulation of MyActuator RMD-X-series
@author
    Tobit Flatscher (github.com/2b-t)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    actuator_parameter_name = 'actuator'
    actuator = LaunchConfiguration(actuator_parameter_name)
    controller_parameter_name = 'controller'
    controller = LaunchConfiguration(controller_parameter_name)
    rqt_controller_manager_parameter_name = 'rqt_controller_manager'
    rqt_controller_manager = LaunchConfiguration(rqt_controller_manager_parameter_name)
    rqt_joint_trajectory_controller_parameter_name = 'rqt_joint_trajectory_controller'
    rqt_joint_trajectory_controller = LaunchConfiguration(rqt_joint_trajectory_controller_parameter_name)

    actuator_cmd = DeclareLaunchArgument(
        actuator_parameter_name,
        default_value='X8ProV2',
        description='Type of the actuator'
    )
    controller_cmd = DeclareLaunchArgument(
        controller_parameter_name,
        choices=['forward_position_controller', 'joint_trajectory_controller'],
        default_value='joint_trajectory_controller',
        description='Controller to be spawned'
    )
    rqt_controller_manager_cmd = DeclareLaunchArgument(
        rqt_controller_manager_parameter_name,
        default_value='true',
        description='Launch rqt controller manager GUI'
    )
    rqt_joint_trajectory_controller_cmd = DeclareLaunchArgument(
        rqt_joint_trajectory_controller_parameter_name,
        default_value='true',
        description='Launch rqt joint trajectory controller GUI'
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
        }.items()
    )

    joint_state_broadcaster_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )
    controller_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[controller, '-c', '/controller_manager']
    )

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch', 'gazebo.launch.py'
                )
            ]
        )
    )
    gazebo_spawner_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='gazebo_spawner',
        arguments=['-entity', 'myactuator_rmd', '-topic', 'robot_description'],
        output='screen'
    )

    rqt_controller_manager_node = Node(
        package='rqt_controller_manager',
        executable='rqt_controller_manager',
        name='rqt_controller_manager',
        output='screen',
        condition=IfCondition(rqt_controller_manager)
    )
    rqt_joint_trajectory_controller_node = Node(
        package='rqt_joint_trajectory_controller',
        executable='rqt_joint_trajectory_controller',
        name='rqt_joint_trajectory_controller',
        output='screen',
        condition=IfCondition(rqt_joint_trajectory_controller)
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
    ld.add_action(controller_cmd)
    ld.add_action(rqt_controller_manager_cmd)
    ld.add_action(rqt_joint_trajectory_controller_cmd)
    ld.add_action(description_launch)
    ld.add_action(joint_state_broadcaster_spawner_node)
    ld.add_action(controller_spawner_node)
    ld.add_action(gazebo_node)
    ld.add_action(gazebo_spawner_node)
    ld.add_action(rqt_controller_manager_node)
    ld.add_action(rqt_joint_trajectory_controller_node)
    ld.add_action(rviz_node)
    return ld
