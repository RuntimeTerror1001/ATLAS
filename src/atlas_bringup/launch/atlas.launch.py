from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction 
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace, SetParameter


def generate_launch_description():
    # Package paths
    atlas_bringup_share = get_package_share_directory('atlas_bringup')
    atlas_state_estimation_share = get_package_share_directory('atlas_state_estimation')
    atlas_slam_sahre  = get_package_share_directory('atlas_slam')
    
    # Launch arguments
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Whether to launch simulation (true) or connect to real robot (false)'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            atlas_bringup_share, 'worlds', 'warehouse_normal.sdf'
        ]),
        description='Path to Gazebo world file'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='atlas',
        description='Robot namespace'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='robot_base_footprint',
        description='Base frame for the robot'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to start RViz visualization'
    )
    
    x_arg = DeclareLaunchArgument('x', default_value='5.0')
    y_arg = DeclareLaunchArgument('y', default_value='5.0') 
    z_arg = DeclareLaunchArgument('z', default_value='0.1')
    robot_arg = DeclareLaunchArgument('robot', default_value='rbkairos')
    robot_model_arg = DeclareLaunchArgument('robot_model', default_value='rbkairos_plus')
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='atlas_robot')

    # 1. Simulation Launch (conditional)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            atlas_bringup_share, '/launch/atlas_sim.launch.py'
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'robot': LaunchConfiguration('robot'),
            'robot_model': LaunchConfiguration('robot_model'),
            'robot_name': LaunchConfiguration('robot_name'),
            'namespace': LaunchConfiguration('namespace'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('sim'))
    )
    
    # 2. State Estimation Launch
    # Group with namespace to ensure all nodes are properly namespaced
    state_estimation_group = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('namespace')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    atlas_state_estimation_share, '/launch/atlas_state_estimation.launch.py'
                ]),
                launch_arguments={
                    'base_frame': LaunchConfiguration('base_frame'),
                    'rviz': LaunchConfiguration('rviz'),
                }.items()
            )
        ]
    )

    # 3. SLAM Launch 
    slam_group = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('namespace')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    atlas_slam_sahre, '/launch/slam_rtabmap.launch.py'
                ]),
                launch_arguments={
                    'base_frame': LaunchConfiguration('base_frame')
                }.items()
            )
        ]
    )
    
    # Future launch includes (commented for now):
    
    # 3. Navigation Launch (Phase 3)
    # navigation_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         get_package_share_directory('atlas_navigation'),
    #         '/launch/navigation.launch.py'
    #     ]),
    #     launch_arguments={
    #         'namespace': LaunchConfiguration('namespace'),
    #         'base_frame': LaunchConfiguration('base_frame'),
    #     }.items()
    # )
    
    # 4. Manipulation Launch (Phase 4) 
    # manipulation_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         get_package_share_directory('atlas_manipulation'),
    #         '/launch/manipulation.launch.py'
    #     ]),
    #     launch_arguments={
    #         'namespace': LaunchConfiguration('namespace'),
    #     }.items()
    # )
    
    # 5. Perception Launch (Phase 5)
    # perception_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         get_package_share_directory('atlas_perception'),
    #         '/launch/perception.launch.py'
    #     ]),
    #     launch_arguments={
    #         'namespace': LaunchConfiguration('namespace'),
    #     }.items()
    # )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        # Launch arguments
        sim_arg,
        world_arg,
        namespace_arg,
        base_frame_arg,
        rviz_arg,
        x_arg, y_arg, z_arg,
        robot_arg, robot_model_arg, robot_name_arg,
        
        # Launch includes
        sim_launch,                    # Phase 1: Simulation
        state_estimation_group,        # Phase 2: State Estimation
        slam_group
        # navigation_launch,           # Phase 3: Navigation (future)
        # manipulation_launch,         # Phase 4: Manipulation (future)  
        # perception_launch,           # Phase 5: Perception (future)
    ])