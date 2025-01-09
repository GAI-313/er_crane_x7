#!/usr/bin/env python3
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    IncludeLaunchDescription
)
from launch_ros.actions import Node, SetParameter
from launch.conditions import IfCondition, UnlessCondition

from er_crane_x7_description.robot_description_loader import RobotDescriptionLoader

from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    ld = LaunchDescription()

    env = {
            'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.environ['LD_LIBRARY_PATH'],
            'IGN_GAZEBO_RESOURCE_PATH': os.path.dirname(
                get_package_share_directory('crane_x7_description'))
        }
    

    ## prefix
    prefix_crane_x7_gazebo = get_package_share_directory('crane_x7_gazebo')
    prefix_er_crane_x7_moveit_config = get_package_share_directory('er_crane_x7_moveit_config')
    prefix_er_crane_x7_config = get_package_share_directory('er_crane_x7_config')
    prefix_er_crane_x7_description = get_package_share_directory('er_crane_x7_description')
    prefix_moveit_config = os.path.join(
        prefix_er_crane_x7_moveit_config, 'config'
    )
    prefix_world_file = os.path.join(
        #prefix_crane_x7_gazebo,
        prefix_er_crane_x7_config,
        'worlds',
        'table.sdf'
    )
    prefix_gui_config = os.path.join(
        prefix_crane_x7_gazebo,
        'gui',
        'gui.config'
    )
    prefix_default_rviz = os.path.join(
        prefix_er_crane_x7_config, 'rviz',
        'sim_viewer.rviz'
    )
    prefix_d435_bridge = os.path.join(
        prefix_er_crane_x7_description, 'config',
        'd435_bridge.yaml'
    )


    ## configurations
    config_world_file = LaunchConfiguration('world_file')
    config_gui_config = LaunchConfiguration('gui_config')
    config_use_d435 = LaunchConfiguration('use_d435')
    config_servo_description = LaunchConfiguration('servo_description')
    config_rviz = LaunchConfiguration('rviz')


    ## declare arguments
    declare_world_file = DeclareLaunchArgument(
        'world_file', default_value=prefix_world_file,
        description=''
    )
    declare_gui_config = DeclareLaunchArgument(
        'gui_config', default_value=prefix_gui_config,
        description=''
    )
    declare_use_d435 = DeclareLaunchArgument(
        'use_d435', default_value='true',
        description=''
    )
    declare_servo_description = DeclareLaunchArgument(
        'servo_description',
        default_value=os.path.join(prefix_moveit_config, 'sim_servo.yaml'),
        description=''
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value=prefix_default_rviz,
        description=''
    )

    ld.add_action(declare_world_file)
    ld.add_action(declare_gui_config)
    ld.add_action(declare_use_d435)
    ld.add_action(declare_servo_description)
    ld.add_action(declare_rviz)


    ## load description
    dl = RobotDescriptionLoader()
    dl.use_gazebo = 'true'
    dl.use_d435 = config_use_d435
    dl.gz_control_config_package = 'crane_x7_control'
    dl.gz_control_config_file_path = 'config/crane_x7_controllers.yaml'
    description = dl.load()


    ## use sim time
    ld.add_action(SetParameter(name='use_sim_time', value=True))


    ## executable actions
    execute_ign_gazebo = ExecuteProcess(
        cmd=['ign gazebo -r', config_world_file, '--gui-config', config_gui_config],
        output='screen',
        additional_env=env,
        shell=True
    )

    execute_spawn_joint_state_controller = ExecuteProcess(
        cmd=['ros2 run controller_manager spawner joint_state_controller'],
        shell=True,
        output='screen',
    )

    execute_spawn_arm_controller = ExecuteProcess(
        cmd=['ros2 run controller_manager spawner crane_x7_arm_controller'],
        shell=True,
        output='screen',
    )

    execute_spawn_gripper_controller = ExecuteProcess(
        cmd=['ros2 run controller_manager spawner crane_x7_gripper_controller'],
        shell=True,
        output='screen',
    )

    node_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/robot_description',
                   '-name', 'crane_x7',
                   '-z', '1.015',
                   '-allow_renaming', 'true'],
    )

    node_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen',
        condition=UnlessCondition(config_use_d435)
    )

    node_gz_d435_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': prefix_d435_bridge}],
        output='screen',
        condition=IfCondition(config_use_d435)
    )

    ld.add_action(execute_ign_gazebo)
    ld.add_action(execute_spawn_joint_state_controller)
    ld.add_action(execute_spawn_arm_controller)
    ld.add_action(execute_spawn_gripper_controller)
    ld.add_action(node_spawn_entity)
    ld.add_action(node_gz_bridge)
    ld.add_action(node_gz_d435_bridge)


    ## executable launch
    launch_moveit_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            prefix_er_crane_x7_moveit_config,
            '/launch/moveit_group_launch.py'
        ]),
        launch_arguments={
            'robot_description': description,
            'rviz' : config_rviz,
            'servo_description' : config_servo_description
        }.items()
    )

    ld.add_action(launch_moveit_group)


    return ld