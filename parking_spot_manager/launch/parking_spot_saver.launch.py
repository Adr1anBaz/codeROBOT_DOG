import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    map_name = LaunchConfiguration('map_name')
    base_directory = LaunchConfiguration('base_directory')
    auto_save_distance = LaunchConfiguration('auto_save_distance')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value=os.getenv('MAP_NAME', 'my_map'),
            description='Name of the map being created'),
        DeclareLaunchArgument(
            'base_directory',
            default_value='',
            description='Directory to save parking spots YAML files'),
        DeclareLaunchArgument(
            'auto_save_distance',
            default_value='0.5',
            description='Auto-save a spot every N meters (0 = disabled)'),

        Node(
            package='parking_spot_manager',
            executable='parking_spot_saver',
            name='parking_spot_saver',
            output='screen',
            parameters=[{
                'map_name': map_name,
                'base_directory': base_directory,
                'auto_save_distance': auto_save_distance,
            }],
        ),
    ])
