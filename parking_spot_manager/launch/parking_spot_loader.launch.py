import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    map_name = LaunchConfiguration('map_name')
    base_directory = LaunchConfiguration('base_directory')
    auto_highlight_nearest = LaunchConfiguration('auto_highlight_nearest')
    error_feedback_rate = LaunchConfiguration('error_feedback_rate')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value=os.getenv('MAP_NAME', 'my_map'),
            description='Name of the map to load parking spots for'),
        DeclareLaunchArgument(
            'base_directory',
            default_value='',
            description='Directory where parking spots YAML files are stored'),
        DeclareLaunchArgument(
            'auto_highlight_nearest',
            default_value='true',
            description='Automatically highlight nearest parking spot'),
        DeclareLaunchArgument(
            'error_feedback_rate',
            default_value='5.0',
            description='Rate (Hz) to publish error feedback (0 = disabled)'),

        Node(
            package='parking_spot_manager',
            executable='parking_spot_loader',
            name='parking_spot_loader',
            output='screen',
            parameters=[{
                'map_name': map_name,
                'base_directory': base_directory,
                'auto_highlight_nearest': auto_highlight_nearest,
                'error_feedback_rate': error_feedback_rate,
            }],
        ),
    ])
