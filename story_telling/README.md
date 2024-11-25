# `story_telling` ROS 2 Package

## Package Description

The `story_telling` package integrates OpenAI's assistant API into ROS 2, enabling an interactive storytelling experience. Designed as part of a larger system for a storytelling robot, this package provides services to set up story parameters, send messages to the assistant, and receive responses. It allows the development of interactive and adaptive narratives in robotics.

### Key Features

- **Interactive Storytelling**: Users can interact with a narrative using ROS 2 services, where OpenAI's assistant generates contextual responses based on user inputs and predefined story settings.
- **Customizable Story Parameters**: Story characteristics such as theme, forbidden topics, and participant details (e.g., age, profession, hobbies) can be specified.
- **Three Core Services**: 
  - `SetupStory`: Configures the story environment with parameters like name, age, theme, etc.
  - `NewMessage`: Sends a new message to the OpenAI assistant for generating responses.
  - `ObtainResponse`: Retrieves the assistant’s response based on the current story context.

### Service Interfaces

The package provides three service interfaces (`srv`):

1. **NewMessage**: Sends a string to the OpenAI assistant as a new message.
    - **Parameters**:
      - `input_text` (string): The message text to send.
    
2. **SetupStory**: Initializes the story context, including participant details, theme, and restricted topics.
    - **Parameters**:
      - `name` (string): Name of the participant.
      - `age` (int32): Age of the participant.
      - `brain` (string): Cognitive condition of the participant (e.g., "none").
      - `hobbies` (string): Relevant hobbies of the participant.
      - `profession` (string): Profession of the participant.
      - `family` (string): Family details.
      - `theme` (string): Story theme (e.g., "adventure in a forest").
      - `forbidden_topics` (string): Topics to avoid in the story (e.g., "violence and sadness").

3. **ObtainResponse**: Retrieves the response from the assistant based on the current story context.
    - **Parameters**: None (relies on context set by previous services).

### Python-Based Implementation

All services are implemented in Python, making it easy for developers to extend and integrate these functionalities into their ROS 2 projects. The primary implementation is in:
- **`story_telling_services.py`**: Handles all services and their logic.

### Launch File Support

The package includes a launch file for easy deployment:
- **`story_telling_launch.py`**: Starts the services together, simplifying setup in ROS 2 applications.

## Dependencies

### ROS 2 Dependencies
- **`ament_cmake`**: Required for building the package.
- **`rclpy`**: The ROS 2 Python client library.
- **`std_msgs`**: Standard message types.
- **`rosidl_default_generators`**: Used for generating service interfaces.

### Python Dependencies
This package also requires the following Python libraries:
- **`openai`**: Official library to interact with OpenAI’s API.

## Example Usage

### Example Python Script
Below is an example of how to use the services in this package:

```python
from story_telling.srv import SetupStory, NewMessage, ObtainResponse
import rclpy

def story_telling_example():
    rclpy.init()
    node = rclpy.create_node('story_telling_example_node')

    # Client for setting up the story
    setup_client = node.create_client(SetupStory, 'setup_story')
    setup_request = SetupStory.Request()
    setup_request.name = "John"
    setup_request.age = 88
    setup_request.brain = "none"
    setup_request.hobbies = "talking, reading"
    setup_request.profession = "student"
    setup_request.family = "sister: Maria, cousin: Dave"
    setup_request.theme = "adventure in a forest"
    setup_request.forbidden_topics = "violence and sadness"

    # Wait for the service to be available
    if not setup_client.service_is_ready():
        setup_client.wait_for_service()
    
    setup_future = setup_client.call_async(setup_request)
    rclpy.spin_until_future_complete(node, setup_future)

    if setup_future.result() is not None:
        print("Story successfully set up!")
    else:
        print("Failed to set up the story.")
        return

    # Client for sending a new message
    new_message_client = node.create_client(NewMessage, 'new_message')
    new_message_request = NewMessage.Request()
    new_message_request.input_text = "Who is the protagonist of the story?"

    if not new_message_client.service_is_ready():
        new_message_client.wait_for_service()
    
    message_future = new_message_client.call_async(new_message_request)
    rclpy.spin_until_future_complete(node, message_future)

    if message_future.result() is not None:
        print("Message successfully sent!")
    else:
        print("Failed to send the message.")
        return

    # Client for obtaining a response
    obtain_response_client = node.create_client(ObtainResponse, 'obtain_response')
    obtain_response_request = ObtainResponse.Request()

    if not obtain_response_client.service_is_ready():
        obtain_response_client.wait_for_service()
    
    response_future = obtain_response_client.call_async(obtain_response_request)
    rclpy.spin_until_future_complete(node, response_future)

    if response_future.result() is not None:
        print("Response received:")
        print(response_future.result().output_text)
    else:
        print("Failed to obtain a response.")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    story_telling_example()
```

## Testing Instructions

You can test the services using ROS 2 terminal commands. For example, to test the `ObtainResponse` service:

```bash
ros2 service call /obtain_response story_telling/srv/ObtainResponse '{}'
```
To test the NewMessage service, you can use the following command:
```bash
ros2 service call /new_message story_telling/srv/NewMessage "{input_text: 'What happens next in the story?'}"
```

To test the SetupStory service, use the following command:
```bash
ros2 service call /setup_story story_telling/srv/SetupStory "{name: 'Alice', age: 86, brain: 'moderate', hobbies: 'reading', profession: 'cook', family: 'son: John', theme: 'adventure in the kitchen', forbidden_topics: 'politics, violence'}"
```

### Launching the Package

To launch the services together, use the provided launch file:

```bash
ros2 launch story_telling story_telling_launch.py
```

This will start all the services in the package, allowing you to interact with the system as defined in the launch file.

## Acknowledgments

Special thanks to:
- **Prof. Doutor Rui P. Rocha** for his continuous guidance, support, and motivation throughout this project.
- **Prof. Doutor Fernando Perdigão** for his for his valuable and critical insights.


## Contact

For any issues or further inquiries, feel free to contact the package maintainers at pedro.almeida@isr.uc.pt.