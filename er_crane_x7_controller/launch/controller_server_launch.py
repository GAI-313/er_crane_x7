#!/usr/bin/nv python3
from launch import LaunchDescription
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction
)
from launch_ros.actions import Node

from er_crane_x7_description.robot_description_loader import RobotDescriptionLoader

from ament_index_python.packages import get_package_share_directory

import os
import yaml


def load_file(file_path):
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(file_path):
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None
    

def context_launch_description(context:LaunchContext, *args, **kwargs):
    ## config
    config_robot_description = LaunchConfiguration('robot_description')
    config_semantic_description = LaunchConfiguration('semantic_description')
    config_joint_limits_description = LaunchConfiguration('joint_limits_description')


    ## parameters
    param_robot_description = {'robot_description': config_robot_description}
    param_semantic_description = {'robot_description_semantic' : load_file(
        context.perform_substitution(
            config_semantic_description
        )
    )}
    param_planning_description = {'robot_description_planning' : load_yaml(
        context.perform_substitution(
            config_joint_limits_description
        )
    )}


    ## node
    return [
        Node(
            package='er_crane_x7_controller',
            executable='move_groupstate_server',
            parameters=[
                param_robot_description,
                param_semantic_description,
                param_planning_description
            ]
        ),
        Node(
            package='er_crane_x7_controller',
            executable='arm_pose_server',
            parameters=[
                param_robot_description,
                param_semantic_description,
                param_planning_description
            ]
        ),
        Node(
            package='er_crane_x7_controller',
            executable='gripper_server',
            parameters=[
                param_robot_description,
                param_semantic_description,
                param_planning_description
            ]
        ),
    ]


def generate_launch_description():
    ld = LaunchDescription()
    dl = RobotDescriptionLoader()


    ## prefix
    prefix_er_crane_x7_moveit_config = get_package_share_directory('er_crane_x7_moveit_config')
    prefix_moveit_config = os.path.join(
        prefix_er_crane_x7_moveit_config, 'config'
    )


    ## declare arguments
    declare_robot_description = DeclareLaunchArgument(
        'robot_description',
        default_value=dl.load(),
        description=''
    )
    declare_semantic_description = DeclareLaunchArgument(
        'semantic_description',
        default_value=os.path.join(prefix_moveit_config, 'crane_x7.srdf'),
        description=''
    )
    declare_joint_limits_description = DeclareLaunchArgument(
        'joint_limits_description',
        default_value=os.path.join(prefix_moveit_config, 'joint_limits.yaml'),
        description=''
    )

    ld.add_action(declare_robot_description)
    ld.add_action(declare_semantic_description)
    ld.add_action(declare_joint_limits_description)


    ld.add_action(
        OpaqueFunction(function=context_launch_description)
    )


    return ld