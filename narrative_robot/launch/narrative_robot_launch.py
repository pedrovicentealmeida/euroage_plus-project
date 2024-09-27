import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription 
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('speech_services'), 'launch', 'speech_services_launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('story_telling'), 'launch', 'story_telling_launch.py')
            )
        ),
        Node(
            package='narrative_robot',
            executable='robot.py',
            name='narrative_robot_node',
            output='screen',
        ),
    ])

