from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the STT (Speech-to-Text) service node
        Node(
            package='speech_services',
            executable='stt_service.py',
            name='stt_service_node',
            output='screen',
        ),
        # Launch the TTS (Text-to-Speech) service node
        Node(
            package='speech_services',
            executable='tts_service.py',
            name='tts_service_node',
            output='screen',
        ),
    ])
