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
    prefix_moveit_config = get_package_share_directory('crane_x7_moveit_config')
    prefix_world_file = os.path.join(
        prefix_crane_x7_gazebo,
        'worlds',
        'table.sdf'
    )
    prefix_gui_config = os.path.join(
        prefix_crane_x7_gazebo,
        'gui',
        'gui.config'
    )


    ## configurations
    config_world_file = LaunchConfiguration('world_file')
    config_gui_config = LaunchConfiguration('gui_config')


    ## declare arguments
    declare_world_file = DeclareLaunchArgument(
        'world_file', default_value=prefix_world_file,
        description=''
    )
    declare_gui_config = DeclareLaunchArgument(
        'gui_config', default_value=prefix_gui_config,
        description=''
    )

    ld.add_action(declare_world_file)
    ld.add_action(declare_gui_config)


    ## load description
    dl = RobotDescriptionLoader()
    dl.use_gazebo = 'true'
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
        output='screen'
    )

    ld.add_action(execute_ign_gazebo)
    ld.add_action(execute_spawn_joint_state_controller)
    ld.add_action(execute_spawn_arm_controller)
    ld.add_action(execute_spawn_gripper_controller)
    ld.add_action(node_spawn_entity)
    ld.add_action(node_gz_bridge)


    ## executable launch
    launch_moveit_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            prefix_moveit_config,
            '/launch/run_move_group.launch.py'
        ]),
        launch_arguments={'loaded_description': description}.items()
    )

    ld.add_action(launch_moveit_group)


    return ld