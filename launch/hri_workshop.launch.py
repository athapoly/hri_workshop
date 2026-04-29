#!/usr/bin/env python3
# ============================================================================
#  hri_workshop.launch.py
#  ----------------------
#  Launches the Limo robot in Gazebo with the HRI workshop world that
#  contains a small walking human actor.
#
#  Usage:
#      ros2 launch hri_workshop hri_workshop.launch.py
#
#  Optional arguments:
#      use_rviz:=true       – also start RViz2 with sensor visualisation
#      headless:=true       – run Gazebo server only (no GUI)
# ============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    # ------------------------------------------------------------------ paths
    pkg_hri_workshop   = get_package_share_directory('hri_workshop')
    pkg_limo_gazebosim = get_package_share_directory('limo_gazebosim')
    pkg_limo_desc      = get_package_share_directory('limo_description')
    pkg_gazebo_ros     = get_package_share_directory('gazebo_ros')

    world_file   = os.path.join(pkg_hri_workshop, 'worlds', 'hri_world.world')
    urdf_file    = os.path.join(pkg_limo_desc, 'urdf', 'limo_four_diff.gazebo')
    rviz_config  = os.path.join(pkg_hri_workshop, 'config', 'hri_workshop.rviz')
    models_path  = os.path.join(pkg_limo_gazebosim, 'models')

    # -------------------------------------------------------- launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_headless  = DeclareLaunchArgument(
        'headless', default_value='False',
        description='Set to true to run Gazebo without a GUI window')

    declare_use_rviz  = DeclareLaunchArgument(
        'use_rviz', default_value='True',
        description='Set to false to skip launching RViz2')

    declare_sim_time  = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock')

    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')

    # ---------------------------------------- make Limo models available to Gz
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=models_path
    )

    # ---------------------------------------------------- Gazebo server + GUI
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(PythonExpression(['not ', headless])),
    )

    # ---------------------------------------------------- Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': use_sim_time,
        }],
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # -------------------------------------------------- spawn Limo in Gazebo
    spawn_limo = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_limo',
        arguments=[
            '-entity', 'limo_gazebosim',
            '-topic',  'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.05', '-Y', '0.0',
        ],
        output='screen',
    )

    # ------------------------------------------- twist watchdog (safety limiter)
    twist_watchdog = Node(
        package='limo_gazebosim',
        executable='twist_watchdog.py',
        name='twist_watchdog',
    )

    # ------------------------------------------------------------------ RViz
    rviz2 = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
    )

    # ---------------------------------------------------- human_detector node
    # Uncomment once students have implemented the node:
    # human_detector = Node(
    #     package='hri_workshop',
    #     executable='human_detector',
    #     name='human_detector',
    #     parameters=[{
    #         'camera_topic': '/limo_camera/image',
    #         'cmd_vel_topic': '/cmd_vel',
    #         'use_sim_time': use_sim_time,
    #     }],
    #     output='screen',
    # )

    # --------------------------------------------------------------- assemble
    return LaunchDescription([
        declare_headless,
        declare_use_rviz,
        declare_sim_time,
        set_model_path,
        gzserver,
        gzclient,
        robot_state_pub,
        joint_state_pub,
        spawn_limo,
        twist_watchdog,
        rviz2,
        # human_detector,   # uncomment after students complete the workshop
    ])
