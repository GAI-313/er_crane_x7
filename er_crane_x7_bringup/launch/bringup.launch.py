#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from er_crane_x7_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    config_file_path = os.path.join(
        get_package_share_directory('crane_x7_control'),
        'config',
        'manipulator_config.yaml'
    )

    links_file_path = os.path.join(
        get_package_share_directory('crane_x7_control'),
        'config',
        'manipulator_links.csv'
    )

    description_loader = RobotDescriptionLoader()
    description_loader.port_name = LaunchConfiguration('port_name')
    description_loader.baudrate = LaunchConfiguration('baudrate')
    description_loader.use_d435 = LaunchConfiguration('use_d435')
    description_loader.timeout_seconds = '1.0'
    description_loader.gen_link = 'true'
    description_loader.manipulator_config_file_path = config_file_path
    description_loader.manipulator_links_file_path = links_file_path

    description = description_loader.load()

    declare_port_name = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyUSB0',
        description='Set port name.'
    )

    declare_baudrate = DeclareLaunchArgument(
        'baudrate',
        default_value='3000000',
        description='Set baudrate.'
    )

    declare_use_d435 = DeclareLaunchArgument(
        'use_d435',
        default_value='true',
        description='Use d435.'
    )
    declare_description = DeclareLaunchArgument(
        'description',
        default_value=description,
        description='Robot description'
    )

    move_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('crane_x7_moveit_config'),
                '/launch/run_move_group.launch.py']),
            condition=UnlessCondition(LaunchConfiguration('use_d435')),
            launch_arguments={
                'loaded_description': LaunchConfiguration("description")
            }.items()
        )

    rviz_config_file = get_package_share_directory(
        'crane_x7_examples') + '/launch/camera_example.rviz'
    move_group_camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('er_crane_x7_moveit_config'),
                '/launch/moveit_group_launch.py']),
            condition=IfCondition(LaunchConfiguration('use_d435')),
            launch_arguments={
                'robot_description': description,
                'rviz_config_file': rviz_config_file
            }.items()
        )

    control_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('crane_x7_control'),
                '/launch/crane_x7_control.launch.py']),
            launch_arguments={'loaded_description': description}.items()
        )

    realsense_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('realsense2_camera'),
                '/launch/rs_launch.py']),
            condition=IfCondition(LaunchConfiguration('use_d435')),
            launch_arguments={
                'camera_namespace': '',
                'device_type': 'd435',
                'pointcloud.enable': 'true',
                'align_depth.enable': 'true',
            }.items()
        )

    return LaunchDescription([
        declare_port_name,
        declare_baudrate,
        declare_use_d435,
        declare_description,
        move_group,
        move_group_camera,
        control_node,
        realsense_node,
    ])