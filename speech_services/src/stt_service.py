#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import azure.cognitiveservices.speech as speechsdk
from std_msgs.msg import UInt8
from speech_services.srv import Stt

class STTServiceNode(Node):
    def __init__(self):
        super().__init__('stt_service_node')

        # Initialize publisher to publish mouth shape (listen)
        self.mouth_pub = self.create_publisher(UInt8, 'mouth_shape', 10)

        # Initialize the Microsoft (Azure) speech recognition configuration
        self.speech_config = speechsdk.SpeechConfig("43afa09c9aff4cc183a2c64622600983", "westeurope")
        self.audio_config = speechsdk.audio.AudioConfig(use_default_microphone=True)

        # Create the service for STT
        self.srv = self.create_service(Stt, 'stt_service', self.handle_stt_service)

        self.get_logger().info("STT service node is ready.")

    def microsoft_stt(self, language: str):
        """STT function using Microsoft Azure Speech API"""
        # Set the language for recognition
        self.speech_config.speech_recognition_language = language
        speech_recognizer = speechsdk.SpeechRecognizer(speech_config=self.speech_config, audio_config=self.audio_config)

        # Publish a mouth shape (listen)
        self.mouth_pub.publish(UInt8(data=10))

        try:
            # Perform speech recognition
            result = speech_recognizer.recognize_once_async().get()
            if result.reason == speechsdk.ResultReason.RecognizedSpeech:
                return result.text
            else:
                return None
        except Exception as e:
            self.get_logger().error(f"Speech recognition error: {e}")
            return None

    def handle_stt_service(self, request, response):
        """Callback to handle STT service request"""
        language = request.language
        try:
            result = self.microsoft_stt(language)
            self.get_logger().info(f"{result}")
            response.text = result if result else "No speech recognized."
        except Exception as e:
            self.get_logger().error(f"STT error: {e}")
            response.text = "STT error."
        return response

def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the STTServiceNode
    stt_service_node = STTServiceNode()

    # Spin the node to keep it alive
    rclpy.spin(stt_service_node)

    # Shutdown
    stt_service_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

