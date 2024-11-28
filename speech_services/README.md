# `speech_services` ROS 2 Package

## Package Description
The `speech_services` package provides a versatile and efficient solution for speech synthesis (TTS) and speech recognition (STT) within ROS 2 systems. By integrating advanced third-party services such as Microsoft Azure, Google Cloud, and Eleven Labs, this package enables robots to perform dynamic speech interactions, enhancing communication and user experience. Additionally, the package supports lip-sync capabilities, aligning synthesized speech with realistic visual feedback for more engaging and human-like interactions.

## Citation
You're free to reuse the source code in this repository provided that its authors' copyright is retained and you cite in your work the following publication:

*Almeida, Pedro V., & Rocha, Rui P. (2025). AI-powered storytelling with a social assistive robot to foster cognitive health in seniors. 11th International Conference on Automation, Robotics, and Applications (ICARA 2025), Zagreb, Croatia.*

## Key Features

### Lip-Sync Capabilities

The package offers cutting-edge lip-sync features that synchronize mouth movements with generated speech for more natural robotic communication:
- **Microsoft TTS**: Uses Microsoft Azure's API to map phonetic sounds to visemes (visual representations of phonemes). This allows speech and mouth movements to be tightly synchronized, providing a more natural and engaging interaction.
- **Google TTS & Eleven Labs**: These services achieve lip-sync by calculating energy levels in audio frames. This approach dynamically aligns the generated speech with corresponding visual cues.

### Service Interfaces

The package provides four service interfaces (`srv`) to support text-to-speech (TTS) and speech-to-text (STT) functionalities:

- **TtsEleven**: Converts text to speech using the Eleven Labs API.
- **TtsGoogle**: Converts text to speech using the Google Text-to-Speech API.
- **TtsMicrosoft**: Converts text to speech using Microsoft Azure's TTS API.
- **Stt**: Transcribes spoken language into text using Microsoft Azure's Speech-to-Text services.

### Python-Based Implementation

All services in the package are implemented in Python, making it easy for developers to extend and integrate these functionalities into their ROS 2 projects. The two main programs are:
- `tts_service.py`: Manages text-to-speech requests.
- `stt_service.py`: Manages speech-to-text requests.

### Launch File Support

A convenient launch file is included to facilitate the simultaneous launch of both TTS and STT services:
- **`speech_services_launch.py`**: This file allows users to start all necessary services at once, simplifying deployment in ROS 2 applications.

## Dependencies

This package requires the following ROS 2 packages:
- **`ament_cmake`**: Required for building the package.
- **`rclpy`**: The ROS 2 Python client library.
- **`std_msgs`**: Contains standard message types.
- **`rosidl_default_generators`**: Used for generating service interfaces.

### Python Dependencies
Additionally, this package relies on the following non-standard Python libraries:
- **`azure.cognitiveservices.speech`**: For Microsoft Azure's cognitive services, facilitating speech synthesis and recognition.
- **`google.cloud`**: Provides access to Google Cloud services for text-to-speech functionalities.
- **`requests`**: For making HTTP requests to external APIs (case of Eleven Labs).
- **`numpy`**: For performing numerical operations and managing array data.
- **`pyaudio`**: Supports audio output, enabling real-time audio processing.
- **`pydub`**: Facilitates audio manipulation and processing, including conversion and editing.

## Service Parameters
### TtsMicrosoft 
The `TtsMicrosoft` service accepts the following parameters:
- `text` (string): Text to convert to speech.
- `language` (string): Language of the speech (e.g., 'en-US').
- `rate` (string): Speed of the speech (e.g., '0', '10').

### TtsGoogle
The `TtsGoogle` service accepts the following parameters:
- `text` (string): Text to convert to speech.
- `language` (string): Language of the speech (e.g., 'pt-PT').

### TtsEleven
The `TtsEleven` service accepts the following parameters:
- `text` (string): text to convert to speech.

### Stt (Speech-to-Text)
The Stt service accepts the following parameters:
- `language` (string): Language of the speech to recognize (e.g., 'en-US').

## Usage Examples

### Example Usage - Script:
```python
from speech_services.srv import TtsMicrosoft
import rclpy

def tts_example():
    rclpy.init()
    node = rclpy.create_node('tts_example_node')
    client = node.create_client(TtsMicrosoft, 'tts_microsoft')

    request = TtsMicrosoft.Request()
    request.text = "Hello, how are you?"
    request.language = "en-US"
    request.rate = "0"

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        print("Speech synthesis succeeded!")
    else:
        print("Failed to generate speech.")
    
    node.destroy_node()
    rclpy.shutdown()
```

### Speech-to-Text Example
```python
from speech_services.srv import Stt
import rclpy

def stt_example():
    rclpy.init()
    node = rclpy.create_node('stt_example_node')
    client = node.create_client(Stt, 'stt_service')

    request = Stt.Request()
    request.language = "en-US"

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        print(f"Recognized text: {future.result().text}")
    else:
        print("Failed to transcribe speech.")
    
    node.destroy_node()
    rclpy.shutdown()
```

## Testing Instructions
You can test the services using ROS 2 terminal commands. For example, to test the Microsoft TTS service:

```bash
ros2 service call /tts_microsoft speech_services/srv/TtsMicrosoft '{text: "your-text", language: "pt-PT or en-US", rate: 0 or 10}'
```
Replace "your-text" with the text you want to convert to speech. You can also change the language to "en-US" or other supported language codes, and adjust the rate to control the speed of the speech ("0" for normal speed, "10" for faster, etc.).

In case of STT you just need to run the following command:
```bash
ros2 service call /stt_service speech_services/srv/Stt '{language: "language"}'
```
You can adjust the language parameter to match the language of the speech you want to transcribe, such as "en-US" for English or "pt-PT" for Portuguese.

## Acknowledgments

Special thanks to:
- **Prof. Rui P. Rocha** ([rprocha@isr.uc.pt](mailto:rprocha@isr.uc.pt)) for his continuous guidance, support, and motivation throughout this project.
- **Prof. Fernando Perdigão** ([fp@deec.uc.pt](mailto:fp@deec.uc.pt)) for his valuable and critical insights.


## Contact

For any issues or further inquiries, feel free to contact the package maintainers at [pedro.almeida@isr.uc.pt](pedro.almeida@isr.uc.pt).
