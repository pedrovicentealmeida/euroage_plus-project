import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the story telling service node
        Node(
            package='story_telling',
            executable='story_telling_services.py',
            name='story_telling_node',
            output='screen',
        ),
    ])

