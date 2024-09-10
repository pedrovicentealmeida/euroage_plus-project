import rospy
import azure.cognitiveservices.speech as speechsdk
from std_msgs.msg import UInt8
from speech_services.srv import stt, sttResponse

# Initialize the global publisher
mouth_pub = None

# Initialize the Microsoft (Azure) speech recognition configuration once
speech_config = speechsdk.SpeechConfig("43afa09c9aff4cc183a2c64622600983", "westeurope")
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
