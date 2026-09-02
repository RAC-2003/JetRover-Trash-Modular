from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trash_modular',
            executable='trash_sorter_pipeline',
            name='trash_sorter_pipeline',
            output='screen',
        ),
    ])
