"""
Atlas State Estimation Launch File

This launch file brings up the complete state estimation pipeline:
1. Robot Localization EKF for sensor fusion
2. TF Health Check node for diagnostics  
3. Optional static transforms (base_footprint -> base_link)
4. Optional RViz visualization

Usage:
  ros2 launch atlas_state_estimation state_estimation.launch.py
  ros2 launch atlas_state_estimation state_estimation.launch.py base_frame:=base_link rviz:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package path
    pkg_share = FindPackageShare('atlas_state_estimation')

    # Launch arguments 
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base frame for the robot (base_link or robot_base_footprint)'
    )

    publish_base_footprint_tf_arg = DeclareLaunchArgument(
        'publish_base_footprint_tf',
        default_value='false',
        description='Whether to publish robot_base_footprint -> base_link transform'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to start RViz with state estimation config'
    )

    ekf_config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'ekf.yaml'
    ])

    rviz_config_file = PathJoinSubstitution([
        pkg_share,
        'rviz',
        'state_estimation.rviz'
    ])

    # Robot Localization Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_file
        ]
    )

    # Optional Static Transform Publisher: robot_base_footprint -> base_link
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'robot_base_footprint', 'base_link'],
        condition=IfCondition(LaunchConfiguration('publish_base_footprint_tf')),
        output='log'
    )

    # TF Health Check Node
    tf_health_check_node = Node(
        package='atlas_state_estimation',
        executable='tf_health_check',
        name='tf_health_check',
        output='screen',
        parameters=[
            {
                'base_frame': LaunchConfiguration('base_frame'),
                'check_frequency': 1.0, # Check TF health at 1Hz
                'timeout': 1.0,
            }
        ]
    )

    # IMU Covariance Fixer Node
    imu_fixer_node = Node(
        package='atlas_state_estimation',
        executable='imu_covariance_fixer.py',
        name='imu_covariance_fixer'
    )

    delayed_imu_fixer = TimerAction(
        period=7.5,
        actions=[imu_fixer_node]
    )

    # RViz Node
    rviz_node = Node(
        package='rivz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='log'
    )

    return LaunchDescription([
        # Launch arguments
        base_frame_arg,
        publish_base_footprint_tf_arg,
        rviz_arg,

        # Nodes
        ekf_node,
        static_tf_node,
        tf_health_check_node,
        delayed_imu_fixer,
        #rviz_node
    ])