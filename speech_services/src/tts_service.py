#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
import requests
import threading
import time
import numpy as np
import pyaudio
from pydub import AudioSegment
from io import BytesIO
from google.cloud import texttospeech
import azure.cognitiveservices.speech as speechsdk

from speech_services.srv import TtsEleven
from speech_services.srv import TtsGoogle
from speech_services.srv import TtsMicrosoft

class TTSServiceNode(Node):
    def __init__(self):
        super().__init__('tts_service_node')

        # Global publisher for mouth shape visemes
        self.mouth_pub = self.create_publisher(UInt8, 'mouth_shape', 10)

        # Azure Speech Configuration
        self.speech_config = speechsdk.SpeechConfig(subscription="43afa09c9aff4cc183a2c64622600983", region="westeurope")

        # Google Speech Configuration
        try:
            self.google_client = texttospeech.TextToSpeechClient()
        except Exception as e:
            self.get_logger().error(f"Error initializing TextToSpeechClient: {e}")

        # Initialize services
        self.create_service(TtsEleven, 'tts_eleven', self.handle_tts_eleven)
        self.create_service(TtsGoogle, 'tts_google', self.handle_tts_google)
        self.create_service(TtsMicrosoft, 'tts_microsoft', self.handle_tts_microsoft)

        self.get_logger().info("TTS services are ready.")

    def eleven_tts(self, text: str) -> None:
        """
        Converts text to speech using Eleven Labs API.
        """
        url = "https://api.elevenlabs.io/v1/text-to-speech/pMsXgVXv3BLzUgSXRplE"
        headers = {
            "Accept": "audio/mpeg",
            "Content-Type": "application/json",
            "xi-api-key": "7e45c9f57e14bbd6d815cbdd4b516ae8"
        }
        data = {
            "text": text,
            "model_id": "eleven_monolingual_v1",
            "voice_settings": {
                "stability": 0.5,
                "similarity_boost": 0.5
            }
        }

        try:
            response = requests.post(url, json=data, headers=headers)
            response.raise_for_status()
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Error with POST request to Eleven API: {e}")
            return

        # Convert the received audio and analyze energy
        audio_array = np.array((AudioSegment.from_mp3(BytesIO(response.content))).get_array_of_samples())
        self.analyze_audio_energy(audio_array, 44100, "en-US")

    def google_tts(self, text: str, language: str) -> None:
        """
        Converts text to speech using Google Cloud TTS API.
        """
        synthesis_input = texttospeech.SynthesisInput(text=text)
        voice = texttospeech.VoiceSelectionParams(
            language_code=language,
            name='en-US-Neural2-G' if language == 'en-US' else 'pt-PT-Wavenet-D',
            ssml_gender=texttospeech.SsmlVoiceGender.FEMALE
        )
        audio_config = texttospeech.AudioConfig(
            audio_encoding=texttospeech.AudioEncoding.LINEAR16,
            sample_rate_hertz=16000
        )
        response = self.google_client.synthesize_speech(
            input=synthesis_input,
            voice=voice,
            audio_config=audio_config
        )

        self.analyze_audio_energy(np.frombuffer(response.audio_content[44:], dtype=np.int16), 16000, language)

    def microsoft_tts(self, text: str, language: str, rate: str) -> None:
        """
        Converts text to speech using Microsoft Azure TTS.
        """
        speech_synthesizer = self.initialize_synthesizer(language)

        ssml_doc = f"""
        <speak version="1.0" xmlns="http://www.w3.org/2001/10/synthesis" xml:lang="{language}">
            <voice name="{self.speech_config.speech_synthesis_voice_name}">
                <prosody rate="{rate}%">
                    {text}
                </prosody>
            </voice>
        </speak>
        """

        # Mapping for viseme publishing
        viseme_map = {0: 0, 1: 20, 2: 17, 3: 11, 4: 18, 5: 18, 6: 21, 7: 16, 8: 22, 9: 17,
                      10: 11, 11: 17, 12: 20, 13: 20, 14: 14, 15: 13, 16: 20, 17: 21,
                      18: 15, 19: 14, 20: 21, 21: 12}

        # Callback function for visemes
        def viseme_cb(evt) -> None:
            viseme_id = viseme_map.get(evt.viseme_id)
            if viseme_id is not None:
                self.mouth_pub.publish(UInt8(data=viseme_id))

        speech_synthesizer.viseme_received.connect(viseme_cb)

        # Synthesize speech using SSML
        try:
            result = speech_synthesizer.speak_ssml_async(ssml_doc).get()
            if result.reason != speechsdk.ResultReason.SynthesizingAudioCompleted:
                self.get_logger().error(f"Error synthesizing speech: {result.reason}")
        except Exception as e:
            self.get_logger().error(f"Error connecting to Azure: {e}")

    def analyze_audio_energy(self, audio_data: np.ndarray, sample_rate: int, language: str) -> None:
        """
        Analyzes audio energy and publishes mouth shape visemes.
        """
        frame_length = 2048
        energy_limits = [1, 30, 100] if sample_rate == 44100 else [10, 25, 50] if language == "en-US" else [15, 40, 100]
        hop_length = 1024 if sample_rate == 44100 else 512

        audio_thread = threading.Thread(target=self.play_audio, args=(audio_data, sample_rate))
        audio_thread.start()

        audio_array_normalized = audio_data.astype(np.float32) / np.max(np.abs(audio_data))

        previous_viseme = None
        for i in range(0, len(audio_array_normalized) - frame_length, hop_length):
            energy_val = sum(abs(audio_array_normalized[i:i + frame_length]) ** 2)
            viseme = 0

            if energy_val <= energy_limits[0]:
                viseme = 0
            elif energy_limits[0] < energy_val < energy_limits[1]:
                viseme = 20
            elif energy_limits[1] <= energy_val < energy_limits[2]:
                viseme = 18
            else:
                viseme = 17

            if viseme != previous_viseme:
                self.mouth_pub.publish(UInt8(data=viseme))
                previous_viseme = viseme

            time.sleep(hop_length / sample_rate)

    def play_audio(self, audio_array: np.ndarray, rate: int) -> None:
        """
        Plays audio.
        """
        p = pyaudio.PyAudio()
        stream = p.open(format=pyaudio.paInt16, channels=1, rate=rate, output=True)
        stream.write(audio_array.astype(np.int16).tobytes())
        stream.close()
        p.terminate()

    def handle_tts_eleven(self, request, response):
        """
        Service handler for Eleven Labs TTS.
        """
        try:
            self.eleven_tts(request.text)
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Error in TTS service (Eleven): {e}")
            response.success = False
        return response

    def handle_tts_google(self, request, response):
        """
        Service handler for Google TTS.
        """
        try:
            self.google_tts(request.text, request.language)
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Error in TTS service (Google): {e}")
            response.success = False
        return response

    def handle_tts_microsoft(self, request, response):
        """
        Service handler for Microsoft TTS.
        """
        try:
            self.microsoft_tts(request.text, request.language, request.rate)
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Error in TTS service (Microsoft): {e}")
            response.success = False
        return response

    def initialize_synthesizer(self, language: str):
        """
        Initializes Azure Speech Synthesizer based on language.
        """
        voice_name = 'en-US-JennyNeural' if language == "en-US" else 'pt-PT-FernandaNeural'
        self.speech_config.speech_synthesis_voice_name = voice_name
        self.speech_config.speech_recognition_language = language

        return speechsdk.SpeechSynthesizer(speech_config=self.speech_config)

def main(args=None):
    rclpy.init(args=args)

    tts_service_node = TTSServiceNode()

    try:
        rclpy.spin(tts_service_node)
    except KeyboardInterrupt:
        tts_service_node.get_logger().info('Shutting down TTS services...')
    finally:
        tts_service_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

