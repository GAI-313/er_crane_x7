#!/usr/bin/env python3
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    LogInfo,
    GroupAction,
    IncludeLaunchDescription
)
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory

from er_crane_x7_description.robot_description_loader import RobotDescriptionLoader

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
    config_kinematics_description = LaunchConfiguration('kinematics_description')
    config_ompl_planning_description = LaunchConfiguration('ompl_planning_description')
    config_controllers_description = LaunchConfiguration('controllers_description')
    config_servo_description = LaunchConfiguration('servo_description')
    config_rviz = LaunchConfiguration('rviz')


    ## parameters
    param_robot_description = {'robot_description': config_robot_description}
    param_ompl_planning_descriptiong = {'move_group': {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization \
                               default_planner_request_adapters/FixWorkspaceBounds \
                               default_planner_request_adapters/FixStartStateBounds \
                               default_planner_request_adapters/FixStartStateCollision \
                               default_planner_request_adapters/FixStartStatePathConstraints',
        'start_state_max_bounds_error': 0.1}}
    param_ompl_planning_descriptiong['move_group'].update(
        load_yaml(
            context.perform_substitution(
                config_ompl_planning_description
            )
        )
    )
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
    param_kinematics_description = load_yaml(
        context.perform_substitution(
            config_kinematics_description
        )
    )
    param_controllers_description = {'moveit_simple_controller_manager' : load_yaml(
            context.perform_substitution(
                config_controllers_description
            )
        ),
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.1
    }
    param_trajectory_execution = {'moveit_manage_controllers': True,
                            'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                            'trajectory_execution.allowed_goal_duration_margin': 0.5,
                            'trajectory_execution.allowed_start_tolerance': 0.1}

    param_planning_scene_monitor = {'publish_planning_scene': True,
                                         'publish_geometry_updates': True,
                                         'publish_state_updates': True,
                                         'publish_transforms_updates': True}
    
    param_servo_description = {'moveit_servo' : load_yaml(
        context.perform_substitution(
            config_servo_description,
        )
    )}


    ## executable actions
    node_move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            param_robot_description,
            param_semantic_description,
            param_planning_description,
            param_kinematics_description,
            param_ompl_planning_descriptiong,
            param_trajectory_execution,
            param_planning_scene_monitor,
            param_controllers_description
        ]
    )

    node_moveit_servo = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        output='screen',
        parameters=[
            param_servo_description,
            param_robot_description,
            param_semantic_description,
            param_kinematics_description
        ]
    )

    node_stratic_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='own_log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link']
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[param_robot_description]
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', config_rviz],
        parameters=[
            param_robot_description,
            param_semantic_description,
            param_ompl_planning_descriptiong,
            param_kinematics_description
        ]
    )


    ## launch
    launch_controller_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('er_crane_x7_controller'),
            '/launch/controller_server_launch.py'
        ]),
        launch_arguments={
            'robot_description' : config_robot_description,
            'semantic_description' : config_semantic_description,
            'joint_limits_description' : config_joint_limits_description
        }.items()
    )


    ## containers
    container = ComposableNodeContainer(
        name="moveit_servo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::JoyToServoPub",
                name="controller_to_servo_node",
            )
        ]
    )


    ## debug message
    loggers = GroupAction(
        actions=[
            LogInfo(msg=["LOAD SEMANTIC_DESCRIPTION:: ", config_semantic_description]),
            LogInfo(msg=["LOAD KINEMATICS DESCRIPTION:: ", config_kinematics_description]),
            LogInfo(msg=["LOAD JOINT LIMITS DESCRIPTION:: ", config_joint_limits_description]),
            LogInfo(msg=["LOAD CONTROLLERS DESCRIPTION:: ", config_controllers_description]),
            LogInfo(msg=["LOAD SERVO YAML:: ", config_servo_description]),
        ]
    )


    return [
        loggers,
        node_move_group, node_moveit_servo, node_robot_state_publisher, node_stratic_tf, node_rviz,
        launch_controller_server,
        container
    ]


def generate_launch_description():
    ld = LaunchDescription()
    dl = RobotDescriptionLoader()


    ## prefix
    prefix_er_crane_x7_moveit_config = get_package_share_directory('er_crane_x7_moveit_config')
    prefix_er_crane_x7_config = get_package_share_directory('er_crane_x7_config')
    prefix_moveit_config = os.path.join(
        prefix_er_crane_x7_moveit_config, 'config'
    )
    prefix_default_rviz = os.path.join(
        prefix_er_crane_x7_config, 'rviz',
        'sim_viewer.rviz'
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
    declare_kinematics_description = DeclareLaunchArgument(
        'kinematics_description',
        default_value=os.path.join(prefix_moveit_config, 'kinematics.yaml'),
        description=''
    )
    declare_ompl_planning_description = DeclareLaunchArgument(
        'ompl_planning_description',
        default_value=os.path.join(prefix_moveit_config, 'ompl_planning.yaml'),
        description=''
    )
    declare_controllers_description = DeclareLaunchArgument(
        'controllers_description',
        default_value=os.path.join(prefix_moveit_config, 'controllers.yaml'),
        description=''
    )
    declare_servo_description = DeclareLaunchArgument(
        'servo_description',
        default_value=os.path.join(prefix_moveit_config, 'servo.yaml'),
        description=''
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value=prefix_default_rviz,
        description=''
    )

    ld.add_action(declare_robot_description)
    ld.add_action(declare_semantic_description)
    ld.add_action(declare_joint_limits_description)
    ld.add_action(declare_kinematics_description)
    ld.add_action(declare_ompl_planning_description)
    ld.add_action(declare_controllers_description)
    ld.add_action(declare_servo_description)
    ld.add_action(declare_rviz)


    ld.add_action(
        OpaqueFunction(function=context_launch_description)
    )


    return ld