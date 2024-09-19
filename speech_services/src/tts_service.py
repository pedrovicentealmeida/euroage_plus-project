import rospy
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

from speech_services.srv import tts_eleven, tts_elevenResponse
from speech_services.srv import tts_google, tts_googleResponse
from speech_services.srv import tts_microsoft, tts_microsoftResponse

# Global publisher for mouth shape visemes
mouth_pub = None

# Azure Speech Configuration
speech_config = speechsdk.SpeechConfig(subscription="43afa09c9aff4cc183a2c64622600983", region="westeurope")

# Google Speech Configuration
try:
    client = texttospeech.TextToSpeechClient()
except Exception as e:
    rospy.logerr(f"Error initializing TextToSpeechClient: {e}")

def eleven_tts(text: str) -> None:
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
        rospy.logerr(f"Error with POST request to Eleven API: {e}")
        return

    # Convert the received audio and analyze energy
    audio_array = np.array((AudioSegment.from_mp3(BytesIO(response.content))).get_array_of_samples())
    analyze_audio_energy(audio_array, 44100, "en-US")

def google_tts(text: str, language: str) -> None:
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
    response = client.synthesize_speech(
        input=synthesis_input,
        voice=voice,
        audio_config=audio_config
    )

    analyze_audio_energy(np.frombuffer(response.audio_content[44:], dtype=np.int16), 16000, language)

def microsoft_tts(text: str, language: str, rate: str) -> None:
    """
    Converts text to speech using Microsoft Azure TTS.
    """
    speech_synthesizer = initialize_synthesizer(language)

    ssml_doc = f"""
    <speak version="1.0" xmlns="http://www.w3.org/2001/10/synthesis" xml:lang="{language}">
        <voice name="{speech_config.speech_synthesis_voice_name}">
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
            mouth_pub.publish(viseme_id)

    speech_synthesizer.viseme_received.connect(viseme_cb)

    # Synthesize speech using SSML
    try:
        result = speech_synthesizer.speak_ssml_async(ssml_doc).get()
        if result.reason != speechsdk.ResultReason.SynthesizingAudioCompleted:
            rospy.logerr(f"Error synthesizing speech: {result.reason}")
    except Exception as e:
        rospy.logerr(f"Error connecting to Azure: {e}")

def analyze_audio_energy(audio_data: np.ndarray, sample_rate: int, language: str) -> None:
    """
    Analyzes audio energy and publishes mouth shape visemes.
    """
    frame_length = 2048
    energy_limits = [1, 30, 100] if sample_rate == 44100 else [10, 25, 50] if language == "en-US" else [15, 40, 100]
    hop_length = 1024 if sample_rate == 44100 else 512

    audio_thread = threading.Thread(target=play_audio, args=(audio_data, sample_rate))
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
            mouth_pub.publish(viseme)
            previous_viseme = viseme

        time.sleep(hop_length / sample_rate)

def play_audio(audio_array: np.ndarray, rate: int) -> None:
    """
    Plays audio.
    """
    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paInt16, channels=1, rate=rate, output=True)
    stream.write(audio_array.astype(np.int16).tobytes())
    stream.close()
    p.terminate()

def handle_tts_eleven(req):
    """
    Service handler for Eleven Labs TTS.
    """
    try:
        eleven_tts(req.text)
        #rospy.loginfo(f"TTS Eleven - Text: {req.text}")
        return tts_elevenResponse(success=True)
    except Exception as e:
        rospy.logerr(f"Error in TTS service (Eleven): {e}")
        return tts_elevenResponse(success=False)

def handle_tts_google(req):
    """
    Service handler for Google TTS.
    """
    try:
        google_tts(req.text, req.language)
        #rospy.loginfo(f"TTS Google - Text: {req.text} | Language: {req.language}")
        return tts_googleResponse(success=True)
    except Exception as e:
        rospy.logerr(f"Error in TTS service (Google): {e}")
        return tts_googleResponse(success=False)

def handle_tts_microsoft(req):
    """
    Service handler for Microsoft TTS.
    """
    try:
        microsoft_tts(req.text, req.language, req.rate)
        #rospy.loginfo(f"TTS Microsoft - Text: {req.text} | Language: {req.language} | Rate: {req.rate}")
        return tts_microsoftResponse(success=True)
    except Exception as e:
        rospy.logerr(f"Error in TTS service (Microsoft): {e}")
        return tts_microsoftResponse(success=False)

def initialize_synthesizer(language: str):
    """
    Initializes Azure Speech Synthesizer based on language.
    """
    voice_name = 'en-US-JennyNeural' if language == "en-US" else 'pt-PT-FernandaNeural'
    speech_config.speech_synthesis_voice_name = voice_name
    speech_config.speech_recognition_language = language

    return speechsdk.SpeechSynthesizer(speech_config=speech_config)

def main():
    global mouth_pub

    rospy.init_node('tts_services')

    # Initialize global publisher
    mouth_pub = rospy.Publisher('mouth_shape', UInt8, queue_size=10)

    # Initialize services for all TTS providers
    rospy.Service('tts_eleven', tts_eleven, handle_tts_eleven)
    rospy.Service('tts_google', tts_google, handle_tts_google)
    rospy.Service('tts_microsoft', tts_microsoft, handle_tts_microsoft)

    rospy.loginfo("TTS services ready.")
    rospy.spin()

if __name__ == "__main__":
    main()
