import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node


def generate_launch_description():

  urg_param_file = os.path.join(
    get_package_share_directory('ususama_slam'), 'config',
    'urg.yaml')
  hokuyo_node = Node(
    package='urg_node', executable='urg_node_driver', output='screen',
    parameters=[urg_param_file]
    )

  mecanum_param_file = os.path.join(get_package_share_directory(
      'ususama_slam'), 'config', 'mecanum.yaml')
  mecanum_node = LifecycleNode(
    name='mecanum',
    package='mecanum_io', executable='mecanum_node', output='screen',
    parameters=[mecanum_param_file]
    )

  ld = LaunchDescription()
  ld.add_action(hokuyo_node)
  ld.add_action(mecanum_node)

  #ld.add_action(mouse_node)
  #ld.add_action(launch_lidar_node)

  return ld