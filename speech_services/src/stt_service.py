##################################################################################
# BSD 3-Clause License
#
# Copyright (c) 2025, Pedro V. Almeida
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##################################################################################

import rospy
import azure.cognitiveservices.speech as speechsdk
from std_msgs.msg import UInt8
from speech_services.srv import stt, sttResponse

# Initialize the global publisher
mouth_pub = None

# Initialize the Microsoft (Azure) speech recognition configuration once
speech_config = speechsdk.SpeechConfig("YOUR-API-KEY", "YOUR-API-REGION")
audio_config = speechsdk.audio.AudioConfig(use_default_microphone=True)

def microsoft_stt(language: str):
    """STT function using Microsoft Azure Speech API"""
    global speech_config, audio_config

    # Set the language for recognition
    speech_config.speech_recognition_language = language
    speech_recognizer = speechsdk.SpeechRecognizer(speech_config=speech_config, audio_config=audio_config)

    # Publish a mouth shape (listen)
    mouth_pub.publish(10)

    try:
        # Perform speech recognition
        result = speech_recognizer.recognize_once_async().get()
        if result.reason == speechsdk.ResultReason.RecognizedSpeech:
            return result.text
        else:
            return None
    except Exception as e:
        rospy.logerr(f"Speech recognition error: {e}")
        return None

def handle_stt_service(req):
    """Callback to handle STT service request"""
    language = req.language
    try:
        result = microsoft_stt(language)
        rospy.loginfo(f"STT transcription: {result}")
        return sttResponse(text=result if result else "No speech recognized.")
    except Exception as e:
        rospy.logerr(f"STT error: {e}")
        return sttResponse(text="STT error.")

def main():
    # Initialize the ROS node and publisher
    global mouth_pub
    rospy.init_node('stt_service_node')
    mouth_pub = rospy.Publisher('mouth_shape', UInt8, queue_size=10)

    # Initialize the STT service
    service = rospy.Service('stt_service', stt, handle_stt_service)

    rospy.loginfo("STT service ready.")
    rospy.spin()

if __name__ == '__main__':
    main()
