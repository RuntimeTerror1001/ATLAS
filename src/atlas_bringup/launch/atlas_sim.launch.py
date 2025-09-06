import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess,
    SetEnvironmentVariable, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, TextSubstitution, Command
)
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # ---------------- Args ----------------
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([
            get_package_share_directory("atlas_bringup"),
            "worlds", "warehouse_normal.sdf",
        ]),
        description="Path to Fortress world SDF",
    )
    robot_arg = DeclareLaunchArgument("robot", default_value="rbkairos")
    robot_model_arg = DeclareLaunchArgument("robot_model", default_value="rbkairos_plus")
    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="atlas_robot")
    ns_arg = DeclareLaunchArgument("namespace", default_value="atlas")  # robot nodes live under /atlas
    x_arg = DeclareLaunchArgument("x", default_value="5.0")
    y_arg = DeclareLaunchArgument("y", default_value="5.0")
    z_arg = DeclareLaunchArgument("z", default_value="0.1")

    world = LaunchConfiguration("world")
    robot = LaunchConfiguration("robot")
    robot_model = LaunchConfiguration("robot_model")
    robot_name = LaunchConfiguration("robot_name")
    namespace = LaunchConfiguration("namespace")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")

    # ---------------- Resource paths ----------------
    rb_desc_share = get_package_share_directory("robotnik_description")
    atlas_share = get_package_share_directory("atlas_bringup")
    rb_sensors_share = get_package_share_directory("robotnik_sensors")
    ur_desc_share = get_package_share_directory("ur_description")
    rq_desc_share = get_package_share_directory('robotiq_description')
    share_parents = [
        os.path.dirname(rb_desc_share),
        os.path.dirname(rb_sensors_share),
        os.path.dirname(ur_desc_share),
        os.path.dirname(atlas_share),
        os.path.dirname(rq_desc_share)
    ]
    resource_paths = ":".join(share_parents)

    env_gz = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_paths)
    env_ign = SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", resource_paths)

    # ---------------- Start Fortress with world ----------------
    script_path = '/home/redpaladin/gpu_launch_scripts/launch_ignition.sh'
    ign_gazebo = ExecuteProcess(
        cmd=[script_path, '-r', world],
        output='screen',
        shell=False
    )

    # ---------------- Publish /robot_description (Ignition toggle on) ----------------
    robot_desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(rb_desc_share, "launch", "robot_description.launch.py")]
        ),
        launch_arguments={
            "robot": robot,
            "robot_model": robot_model,
            "namespace": namespace,
            "gazebo_ignition": "true",
        }.items(),
    )

    # ---------------- Spawn from /robot_description ----------------
    # Use Command(...) to read the param value; sed strips the "String value is: " prefix.
    urdf_string_cmd = Command([
        "bash -lc '",
        'NS="', namespace, '"; ',
        'NODE="/robot_state_publisher"; ',
        'if [ -n "$NS" ]; then NODE="/$NS/robot_state_publisher"; fi; ',
        'ros2 param get "$NODE" robot_description | sed -e "s/^String value is: //" | tr -d "\\n"',
        "'"
    ])

    spawn = TimerAction(
        period=2.0,  # let world + robot_state_publisher come up
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                name="create_entity",
                output="screen",
                arguments=[
                    "-name", robot_name,
                    "-string", urdf_string_cmd,
                    "-x", x, "-y", y, "-z", z,
                ],
            )
        ],
    )

    # ---------------- Bridge ----------------
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="parameter_bridge",
        output="screen",
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',

            # Model Cameras
            '/atlas/front_camera_color/color/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/atlas/front_camera_color/color/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',

            '/world/warehouse/model/atlas_robot/link/robot_base_footprint/sensor/front_camera_depth/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/world/warehouse/model/atlas_robot/link/robot_base_footprint/sensor/front_camera_depth/depth_image/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',

            # Model LiDARs
            '/world/warehouse/model/atlas_robot/link/robot_base_footprint/sensor/front_laser/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/world/warehouse/model/atlas_robot/link/robot_base_footprint/sensor/front_laser/scan/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            '/world/warehouse/model/atlas_robot/link/robot_base_footprint/sensor/rear_laser/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/world/warehouse/model/atlas_robot/link/robot_base_footprint/sensor/rear_laser/scan/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',

            # Model IMU
            '/atlas/imu/data@sensor_msgs/msg/Imu@ignition.msgs.IMU',

            # World Cameras
            '/world/warehouse/model/realsense_mount_0/link/camera_link/sensor/realsense_mount_0_imu/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/world/warehouse/model/realsense_mount_0/link/camera_link/sensor/realsense_mount_0_rgbd/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/world/warehouse/model/realsense_mount_0/link/camera_link/sensor/realsense_mount_0_rgbd/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/world/warehouse/model/realsense_mount_0/link/camera_link/sensor/realsense_mount_0_rgbd/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/world/warehouse/model/realsense_mount_0/link/camera_link/sensor/realsense_mount_0_rgbd/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',

            '/world/warehouse/model/realsense_mount_1/link/camera_link/sensor/realsense_mount_1_imu/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/world/warehouse/model/realsense_mount_1/link/camera_link/sensor/realsense_mount_1_rgbd/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/world/warehouse/model/realsense_mount_1/link/camera_link/sensor/realsense_mount_1_rgbd/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/world/warehouse/model/realsense_mount_1/link/camera_link/sensor/realsense_mount_1_rgbd/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/world/warehouse/model/realsense_mount_1/link/camera_link/sensor/realsense_mount_1_rgbd/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',

            '/world/warehouse/model/realsense_mount_2/link/camera_link/sensor/realsense_mount_2_imu/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/world/warehouse/model/realsense_mount_2/link/camera_link/sensor/realsense_mount_2_rgbd/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/world/warehouse/model/realsense_mount_2/link/camera_link/sensor/realsense_mount_2_rgbd/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/world/warehouse/model/realsense_mount_2/link/camera_link/sensor/realsense_mount_2_rgbd/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/world/warehouse/model/realsense_mount_2/link/camera_link/sensor/realsense_mount_2_rgbd/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',

            '/world/warehouse/model/realsense_mount_3/link/camera_link/sensor/realsense_mount_3_imu/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/world/warehouse/model/realsense_mount_3/link/camera_link/sensor/realsense_mount_3_rgbd/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/world/warehouse/model/realsense_mount_3/link/camera_link/sensor/realsense_mount_3_rgbd/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/world/warehouse/model/realsense_mount_3/link/camera_link/sensor/realsense_mount_3_rgbd/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/world/warehouse/model/realsense_mount_3/link/camera_link/sensor/realsense_mount_3_rgbd/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/atlas/front_camera_color/color/image_raw', '/atlas/front_camera/color/image'),
            ('/atlas/front_camera_color/color/camera_info', '/atlas/front_camera/color/camera_info'),
            ('/world/warehouse/model/atlas_robot/link/robot_base_footprint/sensor/front_camera_depth/depth_image', '/atlas/front_camera/depth/image'),
            ('/world/warehouse/model/atlas_robot/link/robot_base_footprint/sensor/front_camera_depth/depth_image/points', '/atlas/front_camera/depth/points'),
            
            ('/world/warehouse/model/atlas_robot/link/robot_base_footprint/sensor/front_laser/scan', '/atlas/front_lidar/scan'),
            ('/world/warehouse/model/atlas_robot/link/robot_base_footprint/sensor/front_laser/scan/points', '/atlas/front_lidar/points'),
            ('/world/warehouse/model/atlas_robot/link/robot_base_footprint/sensor/rear_laser/scan', '/atlas/rear_lidar/scan'),
            ('/world/warehouse/model/atlas_robot/link/robot_base_footprint/sensor/rear_laser/scan/points', '/atlas/rear_lidar/points'),
            
            ('/world/warehouse/model/realsense_mount_0/link/camera_link/sensor/realsense_mount_0_imu/imu', '/warehouse/camera_0/imu'),
            ('/world/warehouse/model/realsense_mount_0/link/camera_link/sensor/realsense_mount_0_rgbd/camera_info', '/warehouse/camera_0/camera_info'),
            ('/world/warehouse/model/realsense_mount_0/link/camera_link/sensor/realsense_mount_0_rgbd/depth_image', '/warehouse/camera_0/depth/image'),
            ('/world/warehouse/model/realsense_mount_0/link/camera_link/sensor/realsense_mount_0_rgbd/image', '/warehouse/camera_0/color/image'),
            ('/world/warehouse/model/realsense_mount_0/link/camera_link/sensor/realsense_mount_0_rgbd/points', '/warehouse/camera_0/depth/points'),

            ('/world/warehouse/model/realsense_mount_1/link/camera_link/sensor/realsense_mount_1_imu/imu', '/warehouse/camera_1/imu'),
            ('/world/warehouse/model/realsense_mount_1/link/camera_link/sensor/realsense_mount_1_rgbd/camera_info', '/warehouse/camera_1/camera_info'),
            ('/world/warehouse/model/realsense_mount_1/link/camera_link/sensor/realsense_mount_1_rgbd/depth_image', '/warehouse/camera_1/depth/image'),
            ('/world/warehouse/model/realsense_mount_1/link/camera_link/sensor/realsense_mount_1_rgbd/image', '/warehouse/camera_1/color/image'),
            ('/world/warehouse/model/realsense_mount_1/link/camera_link/sensor/realsense_mount_1_rgbd/points', '/warehouse/camera_1/depth/points'),

            ('/world/warehouse/model/realsense_mount_2/link/camera_link/sensor/realsense_mount_2_imu/imu', '/warehouse/camera_2/imu'),
            ('/world/warehouse/model/realsense_mount_2/link/camera_link/sensor/realsense_mount_2_rgbd/camera_info', '/warehouse/camera_2/camera_info'),
            ('/world/warehouse/model/realsense_mount_2/link/camera_link/sensor/realsense_mount_2_rgbd/depth_image', '/warehouse/camera_2/depth/image'),
            ('/world/warehouse/model/realsense_mount_2/link/camera_link/sensor/realsense_mount_2_rgbd/image', '/warehouse/camera_2/color/image'),
            ('/world/warehouse/model/realsense_mount_2/link/camera_link/sensor/realsense_mount_2_rgbd/points', '/warehouse/camera_2/depth/points'),

            ('/world/warehouse/model/realsense_mount_3/link/camera_link/sensor/realsense_mount_3_imu/imu', '/warehouse/camera_3/imu'),
            ('/world/warehouse/model/realsense_mount_3/link/camera_link/sensor/realsense_mount_3_rgbd/camera_info', '/warehouse/camera_3/camera_info'),
            ('/world/warehouse/model/realsense_mount_3/link/camera_link/sensor/realsense_mount_3_rgbd/depth_image', '/warehouse/camera_3/depth/image'),
            ('/world/warehouse/model/realsense_mount_3/link/camera_link/sensor/realsense_mount_3_rgbd/image', '/warehouse/camera_3/color/image'),
            ('/world/warehouse/model/realsense_mount_3/link/camera_link/sensor/realsense_mount_3_rgbd/points', '/warehouse/camera_3/depth/points'),
        ]
    )
    """
/clock
/gazebo/resource_paths
/gui/camera/pose
/sensors/marker
/stats
/world/warehouse/clock
/world/warehouse/dynamic_pose/info

/world/warehouse/model/realsense_mount_0/link/camera_link/sensor/realsense_mount_0_imu/imu
/world/warehouse/model/realsense_mount_0/link/camera_link/sensor/realsense_mount_0_rgbd/camera_info
/world/warehouse/model/realsense_mount_0/link/camera_link/sensor/realsense_mount_0_rgbd/depth_image
/world/warehouse/model/realsense_mount_0/link/camera_link/sensor/realsense_mount_0_rgbd/image
/world/warehouse/model/realsense_mount_0/link/camera_link/sensor/realsense_mount_0_rgbd/points
/world/warehouse/model/realsense_mount_1/link/camera_link/sensor/realsense_mount_1_imu/imu
/world/warehouse/model/realsense_mount_1/link/camera_link/sensor/realsense_mount_1_rgbd/camera_info
/world/warehouse/model/realsense_mount_1/link/camera_link/sensor/realsense_mount_1_rgbd/depth_image
/world/warehouse/model/realsense_mount_1/link/camera_link/sensor/realsense_mount_1_rgbd/image
/world/warehouse/model/realsense_mount_1/link/camera_link/sensor/realsense_mount_1_rgbd/points
/world/warehouse/model/realsense_mount_2/link/camera_link/sensor/realsense_mount_2_imu/imu
/world/warehouse/model/realsense_mount_2/link/camera_link/sensor/realsense_mount_2_rgbd/camera_info
/world/warehouse/model/realsense_mount_2/link/camera_link/sensor/realsense_mount_2_rgbd/depth_image
/world/warehouse/model/realsense_mount_2/link/camera_link/sensor/realsense_mount_2_rgbd/image
/world/warehouse/model/realsense_mount_2/link/camera_link/sensor/realsense_mount_2_rgbd/points
/world/warehouse/model/realsense_mount_3/link/camera_link/sensor/realsense_mount_3_imu/imu
/world/warehouse/model/realsense_mount_3/link/camera_link/sensor/realsense_mount_3_rgbd/camera_info
/world/warehouse/model/realsense_mount_3/link/camera_link/sensor/realsense_mount_3_rgbd/depth_image
/world/warehouse/model/realsense_mount_3/link/camera_link/sensor/realsense_mount_3_rgbd/image
/world/warehouse/model/realsense_mount_3/link/camera_link/sensor/realsense_mount_3_rgbd/points
/world/warehouse/pose/info
/world/warehouse/scene/deletion
/world/warehouse/scene/info
/world/warehouse/state
/world/warehouse/stats

    """
    # ---------------- Controller spawners ----------------
    # Controller manager lives under the same namespace as the robot nodes.
    manager_path = [TextSubstitution(text="/"), namespace, TextSubstitution(text="/controller_manager")]

    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_jsb",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", manager_path,
        ],
    )

    spawner_base = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_base",
        output="screen",
        arguments=[
            "robotnik_base_controller",
            "--controller-manager", manager_path,
        ],
    )

    spawner_arm = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_arm",
        output="screen",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager", manager_path,
        ],
    )

    spawner_gripper = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_gripper",
        output="screen",
        arguments=[
            "gripper_controller",
            "--controller-manager", manager_path,
        ],
    )

    spawn_controllers = TimerAction(
        period=4.5,  # after the entity exists and controller_manager is ready
        actions=[spawner_jsb, spawner_base, spawner_arm, spawner_gripper],
    )

    return LaunchDescription([
        world_arg, robot_arg, robot_model_arg, robot_name_arg, ns_arg, x_arg, y_arg, z_arg,
        env_gz, env_ign,
        ign_gazebo,
        robot_desc_launch,
        spawn,
        bridge,
        spawn_controllers,
    ])
