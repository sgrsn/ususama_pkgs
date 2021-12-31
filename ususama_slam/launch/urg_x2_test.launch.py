import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    param_file_A = os.path.join(
            get_package_share_directory('ususama_slam'), 'config',
            'urg_A.yaml')
    hokuyo_node_A = Node(
        package='urg_node', executable='urg_node_driver', output='screen',
        parameters=[param_file_A]
        )
        
    param_file_B = os.path.join(
            get_package_share_directory('ususama_slam'), 'config',
            'urg_B.yaml')
    hokuyo_node_B = Node(
        package='urg_node', executable='urg_node_driver', output='screen',
        parameters=[param_file_B]
        )

    ld.add_action(hokuyo_node_A)
    ld.add_action(hokuyo_node_B)
    return ld