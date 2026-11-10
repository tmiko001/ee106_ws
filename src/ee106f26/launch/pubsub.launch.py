from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ee106f26',
            executable='publisher.py',
            name='talker',
            output='screen',
        ),
        Node(
            package='ee106f26',
            executable='subscriber.py',
            name='listener',
            output='screen',
        ),
    ])
