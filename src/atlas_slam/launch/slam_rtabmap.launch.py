from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node 
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    atlas_slam_share = FindPackageShare('atlas_slam')

    # Frame & Topic Arguments
    base_frame_arg = DeclareLaunchArgument('base_frame', default_value='robot_base_footprint')
    odom_frame_arg = DeclareLaunchArgument('odom_frame', default_value='robot_odom')
    map_frame_arg = DeclareLaunchArgument('map_frame', default_value='robot_map')

    rgb_topic_arg = DeclareLaunchArgument('rgb_topic', default_value='/atlas/front_camera/color/image')
    depth_topic_arg = DeclareLaunchArgument('depth_topic', default_value='/atlas/front_camera/depth/image')
    caminfo_topic_arg = DeclareLaunchArgument('caminfo_topic', default_value='/atlas/front_camera/color/camera_info')
    scan_cloud_topic_arg = DeclareLaunchArgument('scan_cloud_topic', default_value='/atlas/front_lidar/points')

    # 1) Pack RGB + Depth into single RGBD Image
    rgbd_sync = Node(
        package='rtabmap_sync', executable='rgbd_sync',  name='rgbd_sync',
        parameters=[{
            'approx_sync': True,
            'sync_queue_size': 30,
            'use_sim_time': True
        }],
        remappings=[
            ('rgb/image', LaunchConfiguration('rgb_topic')),
            ('depth/image', LaunchConfiguration('depth_topic')),
            ('rgb/camera_info', LaunchConfiguration('caminfo_topic')),
            # Output
            ('rgbd_image', 'rtabmap/rgbd_image')
        ],
        output='screen'
    )

    # 2) Visual Odometry from RGB-D
    rgbd_odom = Node(
        package='rtabmap_odom', executable='rgbd_odometry', name='rgbd_odometry',
        parameters=[{
            'subscribe_rgbd': True,
            'publish_tf': False,
            'wait_for_transform':  0.2,
            'frame_id': LaunchConfiguration('base_frame'),
            'odom_frame_id': LaunchConfiguration('odom_frame'),
            'approx_sync': True,
            'sync_queue_size': 30,
            'use_sim_time': True,

            # Odom Defaults for D435 Indoors
            'Odom/GuessMotion': 'true',
            'Odom/Holonomic': 'true',
            'GFTT/MinDistance': '7',
            'Vis/MinInliers': '12',
            'Vis/EstimationType': '1'
        }],
        remappings=[
            ('rgbd_image', 'rtabmap/rgbd_image'),
            # output:
            ('odom', 'rtabmap/odom')
        ],
        output='screen'
    )

    # 3) RTAB-Map (publishes robot_map -> robot_odom)
    rtabmap = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap',
        parameters=[{
            'use_sim_time': True,

            # Frames
            'frame_id':        LaunchConfiguration('base_frame'),
            'odom_frame_id':   LaunchConfiguration('odom_frame'),
            'map_frame_id':    LaunchConfiguration('map_frame'),
            'publish_tf': True,                    # map->odom by RTAB-Map only
            'tf_delay': 0.05,

            # Subscriptions
            'subscribe_rgbd': True,
            'subscribe_scan': False,
            'subscribe_scan_cloud': True,
            'approx_sync': True,
            'sync_queue_size': 30,

            # Build a useful 2-D grid for Nav2:
            'Grid/FromDepth': 'true',
            'Grid/RangeMin': '0.2',
            'Grid/RangeMax': '10.0',
            'Grid/CellSize': '0.05',
            'Grid/NormalsK': '15',
            'Grid/MaxGroundAngle': '45',
            'Grid/MinClusterSize': '10',

            # LiDAR map integration (downsample to keep it light)
            'scan_cloud_max_points': 40000,
            'scan_voxel_size': 0.10,

            # Mapping behavior
            'Mem/UseOdomInfo': 'true',
            'RGBD/LinearUpdate': '0.2',
            'RGBD/AngularUpdate': '0.15',
            'RGBD/ProximityBySpace': 'true',
            'Kp/MaxDepth': '8.0',
        }],
        remappings=[
            # Inputs
            ('rgbd_image', 'rtabmap/rgbd_image'),
            ('scan_cloud', LaunchConfiguration('scan_cloud_topic')),
            ('odom',       'rtabmap/odom'),
        ],
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz_rtabmap',
        arguments=['-d', PathJoinSubstitution([atlas_slam_share, 'config', 'rtabmap.rviz'])],
        output='screen'
    )

    return LaunchDescription([
        base_frame_arg, odom_frame_arg, map_frame_arg,
        rgb_topic_arg, depth_topic_arg, caminfo_topic_arg, scan_cloud_topic_arg,
        rgbd_sync, rgbd_odom, rtabmap, rviz
    ])