import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_device', default_value='/dev/video0', description='Camera device path'),
        
        launch_ros.actions.Node(
            package='camera_node',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{'camera_device': LaunchConfiguration('camera_device')}],
        ),
    ])
