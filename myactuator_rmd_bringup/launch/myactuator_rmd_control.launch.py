"""
@file myactuator_rmd_control.launch.py
@brief
    Launch file for controlling MyActuator RMD-X-series either with hardware or in simulation
@author
    Tobit Flatscher (github.com/2b-t)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
)
from launch_ros.actions import Node


def generate_launch_description():
    actuator_parameter_name = 'actuator'
    actuator = LaunchConfiguration(actuator_parameter_name)
    actuator_id_parameter_name = 'actuator_id'
    actuator_id = LaunchConfiguration(actuator_id_parameter_name)
    ifname_parameter_name = 'ifname'
    ifname = LaunchConfiguration(ifname_parameter_name)
    extra_status_refresh_rate_parameter_name = 'extra_status_refresh_rate'
    extra_status_refresh_rate = LaunchConfiguration(extra_status_refresh_rate_parameter_name)
    controller_parameter_name = 'controller'
    controller = LaunchConfiguration(controller_parameter_name)
    rqt_controller_manager_parameter_name = 'rqt_controller_manager'
    rqt_controller_manager = LaunchConfiguration(rqt_controller_manager_parameter_name)
    rqt_joint_trajectory_controller_parameter_name = 'rqt_joint_trajectory_controller'
    rqt_joint_trajectory_controller = LaunchConfiguration(rqt_joint_trajectory_controller_parameter_name)
    simulation_parameter_name = 'simulation'
    simulation = LaunchConfiguration(simulation_parameter_name)
    xacro_file_parameter_name = 'xacro_file'
    xacro_file = LaunchConfiguration(xacro_file_parameter_name)

    actuator_cmd = DeclareLaunchArgument(
        actuator_parameter_name,
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
    extra_status_refresh_rate_cmd = DeclareLaunchArgument(
        extra_status_refresh_rate_parameter_name,
        default_value='0',
        description='Extra status refresh'
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
    xacro_file_parameter_cmd = DeclareLaunchArgument(
        xacro_file_parameter_name,
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
            'actuator_id:=', actuator_id, ' ',
            'extra_status_refresh_rate:=', extra_status_refresh_rate
        ]
    )
    robot_description = {'robot_description': robot_description_content}
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
            'simulation': simulation
        }.items()
    )

    joint_state_broadcaster_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )
    myactuator_rmd_state_broadcaster_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['myactuator_rmd_state_broadcaster', '--controller-manager', '/controller_manager'],
        condition=IfCondition(PythonExpression([extra_status_refresh_rate, ' != 0']))
    )
    controllers = PathJoinSubstitution(
        [
            get_package_share_directory('myactuator_rmd_description'),
            'config',
            'myactuator_rmd_controllers.yaml',
        ]
    )
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers],
        output='screen',
        condition=UnlessCondition(simulation)
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
        ),
        condition=IfCondition(simulation)
    )
    gazebo_spawner_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='gazebo_spawner',
        arguments=['-entity', 'myactuator_rmd', '-topic', 'robot_description'],
        output='screen',
        condition=IfCondition(simulation)
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
    ld.add_action(actuator_id_cmd)
    ld.add_action(ifname_cmd)
    ld.add_action(extra_status_refresh_rate_cmd)
    ld.add_action(controller_cmd)
    ld.add_action(rqt_controller_manager_cmd)
    ld.add_action(rqt_joint_trajectory_controller_cmd)
    ld.add_action(simulation_parameter_cmd)
    ld.add_action(xacro_file_parameter_cmd)
    ld.add_action(description_launch)
    ld.add_action(joint_state_broadcaster_spawner_node)
    ld.add_action(myactuator_rmd_state_broadcaster_spawner_node)
    ld.add_action(controller_manager_node)
    ld.add_action(controller_spawner_node)
    ld.add_action(gazebo_node)
    ld.add_action(gazebo_spawner_node)
    ld.add_action(rqt_controller_manager_node)
    ld.add_action(rqt_joint_trajectory_controller_node)
    ld.add_action(rviz_node)
    return ld
