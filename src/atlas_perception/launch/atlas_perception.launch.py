from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package path
    pkg_share = FindPackageShare('atlas_perception')
    
    # Declare arguments
    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/atlas/front_camera/color/image',
        description='RGB image topic'
    )
    
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/atlas/front_camera/depth/image',
        description='Depth image topic'
    )
    
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/atlas/front_camera/color/camera_info',
        description='Camera info topic'
    )
    
    frame_out_arg = DeclareLaunchArgument(
        'frame_out',
        default_value='robot_map',
        description='Default output frame for detections'
    )
    
    publish_debug_images_arg = DeclareLaunchArgument(
        'publish_debug_images',
        default_value='false',
        description='Whether to publish debug images'
    )
    
    # Parameters file
    params_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'params.yaml'
    ])
    
    # Shape detector node
    shape_detector_node = Node(
        package='atlas_perception',
        executable='shape_detector_node',
        name='shape_detector',
        output='screen',
        parameters=[
            params_file,
            {
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'publish_debug_images': LaunchConfiguration('publish_debug_images'),
            }
        ],
    )
    
    return LaunchDescription([
        rgb_topic_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        frame_out_arg,
        publish_debug_images_arg,
        shape_detector_node,
    ])