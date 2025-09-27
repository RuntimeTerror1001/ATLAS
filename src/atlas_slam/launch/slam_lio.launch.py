from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    atlas_slam_share = FindPackageShare('atlas_slam')

    # Launch args
    base_frame_arg = DeclareLaunchArgument('base_frame', default_value='robot_base_footprint')
    odom_frame_arg = DeclareLaunchArgument('odom_frame', default_value='robot_odom')
    map_frame_arg  = DeclareLaunchArgument('map_frame',  default_value='map')  # unify on 'map'
    imu_topic_arg  = DeclareLaunchArgument('imu_topic',  default_value='/atlas/imu')
    points_topic_arg = DeclareLaunchArgument('points_topic', default_value='/atlas/front_lidar/points')

    params_path = PathJoinSubstitution([atlas_slam_share, 'config', 'lio_sam.yaml'])

    # LIO-SAM 
    lio_params = [
        params_path,
        {
            'mapFrame':       LaunchConfiguration('map_frame'),
            'odomFrame':      LaunchConfiguration('odom_frame'),
            'baselinkFrame':  LaunchConfiguration('base_frame'),
            'imuTopic':       LaunchConfiguration('imu_topic'),
            'pointCloudTopic':LaunchConfiguration('points_topic'),
            'use_sim_time':   True
        }
    ]

    lio_group = GroupAction(actions=[
        # Node(package='tf2_ros', executable='static_transform_publisher', name='map_to_odom',
        #     arguments=['0','0','0','0','0','0','robot_map','robot_odom']
        # ),
        Node(package='lio_sam', executable='lio_sam_imuPreintegration', name='lio_sam_imuPreintegration', parameters=lio_params, output='screen'),
        Node(package='lio_sam', executable='lio_sam_imageProjection',   name='lio_sam_imageProjection',   parameters=lio_params, output='screen'),
        Node(package='lio_sam', executable='lio_sam_featureExtraction', name='lio_sam_featureExtraction', parameters=lio_params, output='screen'),
        Node(package='lio_sam', executable='lio_sam_mapOptimization',   name='lio_sam_mapOptimization',   parameters=lio_params, output='screen'),
    ])

    # Relays (unique names)
    odom_relay = Node(
        package='topic_tools', executable='relay', name='lio_odom_relay',
        arguments=['/lio_sam/mapping/odometry', '/atlas/lio/odom'], output='screen'
    )
    path_relay = Node(
        package='topic_tools', executable='relay', name='lio_path_relay',
        arguments=['/lio_sam/mapping/path', '/atlas/lio/path'], output='screen'
    )

    # PC2 fixer (stamps LiDAR frame)
    pc2_fixer_node = Node(
        package='atlas_slam', executable='pc2_fixer', name='pc2_fixer_node',
        parameters=[{
            'input_cloud':  '/world/warehouse/model/atlas_robot/link/robot_base_footprint/sensor/front_laser/scan/points',
            'output_cloud': LaunchConfiguration('points_topic'),
            'frame_out':    'robot_front_laser_link',
            'min_v_deg': -15.0, 'max_v_deg': 15.0, 'vertical_rings': 16, 'scan_rate_hz': 10.0
        }], output='screen'
    )

    # Single, optional RViz
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz_slam',
        arguments=['-d', PathJoinSubstitution([atlas_slam_share, 'config', 'lio_sam.rviz'])],
        output='screen'
    )

    return LaunchDescription([
        base_frame_arg, odom_frame_arg, map_frame_arg, imu_topic_arg, points_topic_arg,
        pc2_fixer_node, lio_group, odom_relay, path_relay, rviz_node
    ])
