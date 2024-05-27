#!/usr/bin/env python3
"""
Author: Tobit Flatscher (github.com/2b-t)

Launch file for loading MyActuator RMD-X-series URDF description
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node


def generate_launch_description():
    actuator_parameter_name = 'actuator'
    actuator = LaunchConfiguration(actuator_parameter_name)
    actuator_id_parameter_name = 'actuator_id'
    actuator_id = LaunchConfiguration(actuator_id_parameter_name)
    ifname_parameter_name = 'ifname'
    ifname = LaunchConfiguration(ifname_parameter_name)
    simulation_parameter_name = 'simulation'
    simulation = LaunchConfiguration(simulation_parameter_name)
    xacro_parameter_name = 'xacro_file'
    xacro_file = LaunchConfiguration(xacro_parameter_name)

    actuator_cmd = DeclareLaunchArgument(
        actuator_parameter_name,
        choices=['X8ProV2', 'X12_150'],
        default_value='X8ProV2',
        description='Type of the actuator'
    )
    actuator_id_cmd = DeclareLaunchArgument(
        actuator_id_parameter_name,
        default_value='1',
        description='Actuator id for real hardware'
    )
    ifname_cmd = DeclareLaunchArgument(
        ifname_parameter_name,
        default_value='can0',
        description='CAN interface name'
    )
    simulation_parameter_cmd = DeclareLaunchArgument(
        simulation_parameter_name,
        default_value='true',
        description='Simulated or real hardware interface'
    )
    default_xacro_file = PathJoinSubstitution(
        [
            get_package_share_directory('myactuator_rmd_description'),
            'urdf', 'standalone.urdf.xacro'
        ]
    )
    xacro_parameter_cmd = DeclareLaunchArgument(
        xacro_parameter_name,
        default_value=default_xacro_file,
        description='Xacro URDF description to be used'
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution([xacro_file]), ' ',
            'actuator:=', actuator, ' ',
            'simulation:=', simulation, ' ',
            'ifname:=', ifname, ' ',
            'actuator_id:=', actuator_id
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    ld = LaunchDescription()
    ld.add_action(actuator_cmd)
    ld.add_action(actuator_id_cmd)
    ld.add_action(ifname_cmd)
    ld.add_action(simulation_parameter_cmd)
    ld.add_action(xacro_parameter_cmd)
    ld.add_action(robot_state_publisher)
    return ld
