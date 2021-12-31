import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('ususama_navigation2'),
            'map',
            'map.yaml'))

    param_dir = LaunchConfiguration(
        'params',
        default=os.path.join(
            get_package_share_directory('ususama_navigation2'),
            'config',
            'ususama_urg04lx.yaml'))

    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            #PythonLaunchDescriptionSource([nav2_launch_dir, '/nav2_bringup_launch.py']),
            PythonLaunchDescriptionSource([nav2_launch_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params': param_dir}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher', output='screen',
            arguments=['0.165', '0', '0.1', '3.14', '3.14', '0.0', 'base_link', 'laser']),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'laser', 'base_laser']),
    ])