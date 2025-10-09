from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    atlas_nav_share = get_package_share_directory('atlas_navigation')
    
    behavior_server_params = os.path.join(atlas_nav_share, 'config', 'behavior_server.yaml')
    bt_nav_params = os.path.join(atlas_nav_share, 'config', 'bt_navigator.yaml')
    controller_server_params = os.path.join(atlas_nav_share, 'config', 'controller_server.yaml')
    g_costmap_params = os.path.join(atlas_nav_share, 'config', 'global_costmap.yaml')
    l_costmap_params = os.path.join(atlas_nav_share, 'config', 'local_costmap.yaml')
    planner_server_params = os.path.join(atlas_nav_share, 'config', 'planner_server.yaml')
    smoother_server_params = os.path.join(atlas_nav_share, 'config', 'smoother_server.yaml')
    velocity_smoother_params = os.path.join(atlas_nav_share, 'config', 'velocity_smoother.yaml')

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[behavior_server_params],
    ) 

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_nav_params],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_server_params, l_costmap_params],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_server_params, g_costmap_params],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[smoother_server_params],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[velocity_smoother_params],
        remappings=[
            ('cmd_vel', 'cmd_vel'),
            ('cmd_vel_smoothed', 'robotnik_base_controller/reference_unstamped')
        ]
    )

    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'bt_navigator',
                'behavior_server',
                'smoother_server',
                'velocity_smoother',
            ]
        }]
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz_nav',
        arguments=['-d', PathJoinSubstitution([atlas_nav_share, 'config', 'nav.rviz'])],
        output='screen'
    )

    return LaunchDescription([
        controller_server,
        planner_server,
        bt_navigator,
        behavior_server,
        smoother_server,
        velocity_smoother,
        lifecycle,
        rviz
    ])