# `speech_services`

## Package Description
The `speech_services` ROS 2 package provides a robust node for speech synthesis and recognition tailored for robotic applications. This package encompasses a range of server implementations that leverage advanced third-party text-to-speech (TTS) and speech-to-text (STT) technologies. By doing so, it allows developers to seamlessly integrate sophisticated voice capabilities into their robotic systems. Moreover, through STT, the robot can accurately transcribe spoken language, enhancing interactive communication with users.

### Lip-Sync Capabilities

The `speech_services` package provides advanced lip-sync functionalities for text-to-speech (TTS) services, significantly enhancing the realism of synthesized speech interactions.

- **Microsoft TTS**: This service achieves lip synchronization by mapping visemes—visual representations of phonemes—to their corresponding phonetic sounds using the Microsoft Azure API. This methodology ensures that the visual depiction of speech aligns closely with the spoken output, resulting in a more natural and engaging interaction.

- **Google TTS and Eleven Labs**: Both services implement lip synchronization by assigning visemes based on energy calculations over audio frames. This technique facilitates dynamic alignment between the generated speech and the visual cues.

## Features

### Service Interfaces
The package defines four service interfaces (`srv`) designed for TTS and STT functionalities:
- **TtsEleven**: Generates speech from text using the Eleven Labs API.
- **TtsGoogle**: Converts text to speech utilizing Google Text-to-Speech services.
- **TtsMicrosoft**: Produces speech from text via Microsoft Azure's TTS capabilities.
- **Stt**: Transforms spoken language into text, leveraging Microsoft Azure's Speech-to-Text technology.

### Python Implementation
All services are implemented in Python, making them accessible and straightforward to extend for developers familiar with the language. The package includes two primary programs:
- `tts_service.py`: Manages text-to-speech requests.
- `stt_service.py`: Handles speech-to-text requests.

### Launch File Support
The package supports launch file integration, facilitating easy deployment of services within ROS 2 applications:
- **`speech_services_launch.py`**: This launch file enables the simultaneous start of both STT and TTS services, allowing usage via terminal commands or scripts in the respective format.

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


## Installation Instructions
To install the `speech_services` package, follow these steps...

## Usage Examples
### Text-to-Speech Example
...

### Speech-to-Text Example
....

## Testing Instructions
...

## Contact
....

## Acknowledgments
.....
