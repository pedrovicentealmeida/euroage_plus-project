from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare the model argument (can be overridden at launch time)
        DeclareLaunchArgument(
            'model',
            default_value='gpt-4o-mini',
            description='OpenAI model to use for story telling'
        ),
        # Launch the story telling service node
        Node(
            package='story_telling',
            executable='story_telling_services.py',
            name='story_telling_node',
            output='screen',
            parameters=[{'model': LaunchConfiguration('model')}]
        ),
    ])
