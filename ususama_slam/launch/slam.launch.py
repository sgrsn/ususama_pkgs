from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    slam_node = Node(
        package='slam_toolbox', executable='sync_slam_toolbox_node',
        output='screen',
        parameters=[
            get_package_share_directory(
                'ususama_slam')
            + '/config/mapper_params_offline.yaml'
        ],
    )

    rviz2_node = Node(
        name='rviz2',
        package='rviz2', executable='rviz2', output='screen',
        arguments=[
            '-d',
            get_package_share_directory('ususama_slam')
            + '/config/default.rviz'],
    )

    #teleop_key_node = Node(
    #    name='teleop_twist_keyboard',
    #    package='teleop_twist_keyboard', executable='teleop_twist_keyboard', output='screen',
    #    arguments=[],
    #)

    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher', output='screen',
        arguments=['0.165', '0', '0.1', '3.14', '3.14', '0.0', 'base_footprint', 'laser'],
    )

    ld = LaunchDescription()
    ld.add_action(slam_node)
    ld.add_action(rviz2_node)
    #ld.add_action(teleop_key_node)
    ld.add_action(static_transform_publisher_node)

    return ld