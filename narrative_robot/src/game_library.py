import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from speech_services.srv import TtsMicrosoft, Stt
from story_telling.srv import NewMessage, SetupStory, ObtainResponse

import queue
import threading

class StoryTelling(Node):
    """Class to handle storytelling with OpenAI assistant."""

    def __init__(self):
        """Initialize the StoryTelling class."""
        super().__init__('story_telling_interactive_node')
        
        self.synthesize_queue = queue.Queue()
        self.queue_empty_event = threading.Event()
        self.synthesize_thread = threading.Thread(target=self.synthesize_text)
        self.synthesize_thread.daemon = True
        self.synthesize_thread.start()

        # Subscribing to the 'story_telling_text' topic
        self.subscription = self.create_subscription(String, 'story_telling_text', self.story_callback, 10)

        self.get_logger().info("Subscribed to 'story_telling_text' topic.")

    def synthesize_text(self):
        """Thread method to process text-to-speech requests."""
        self.get_logger().info("Waiting for TTS Microsoft service...")
        self.client_tts = self.create_client(TtsMicrosoft, 'tts_microsoft')
        while not self.client_tts.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service tts_microsoft not available, waiting...')

        try:
            while rclpy.ok():
                try:
                    text = self.synthesize_queue.get(timeout=1)
                    if text is None:
                        break

                    tts_req = TtsMicrosoft.Request(text=text, language="pt-PT", rate="0")
                    self.get_logger().info(f"Synthesizing speech: {text}")
                    self.client_tts.call(tts_req)
                    self.synthesize_queue.task_done()

                    if self.synthesize_queue.empty():
                        self.queue_empty_event.set()

                except queue.Empty:
                    continue

        except Exception as e:
            self.get_logger().error(f"Unexpected error in synthesize_text: {e}")

    def story_callback(self, msg):
        """Callback function that gets called whenever a message is published on the topic."""
        self.synthesize_queue.put(msg.data)
        self.queue_empty_event.clear()

    def new_message(self):
        """Handles sending a new message to the story-telling service."""
        stt_response = self.call_stt_service()
        if stt_response:
            self.call_new_message_service(stt_response.text)
            return stt_response.text

    def call_stt_service(self):
        """Call the STT service and return the response."""
        self.get_logger().info("Waiting for STT service...")
        self.client_stt = self.create_client(Stt, 'stt_service')
        while not self.client_stt.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service stt_service not available, waiting...')

        try:
            req = Stt.Request(language="pt-PT")
            self.get_logger().info("Requesting speech-to-text...")
            return self.client_stt.call(req)
        except Exception as e:
            self.get_logger().error(f"STT service call failed: {e}")
            return None

    def call_new_message_service(self, text):
        """Call the new_message service with the provided text."""
        self.get_logger().info("Waiting for New Message service...")
        self.client_new_message = self.create_client(NewMessage, 'new_message')
        while not self.client_new_message.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service new_message not available, waiting...')

        try:
            req = NewMessage.Request(input_text=text)
            self.get_logger().info(f"Sending new message: {text}")
            response = self.client_new_message.call(req)
            self.get_logger().info(f"New message service response: {response}")
        except Exception as e:
            self.get_logger().error(f"New message service call failed: {e}")

    def define_parameters(self, name, age, brain, hobbies, profession, family, theme, forbidden_topics):
        """Define story parameters by calling the setup_story service."""
        self.get_logger().info("Waiting for Setup Story service...")
        self.client_setup_story = self.create_client(SetupStory, 'setup_story')
        while not self.client_setup_story.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service setup_story not available, waiting...')

        try:
            req = SetupStory.Request(
                name=name, age=age, brain=brain, hobbies=hobbies,
                profession=profession, family=family, theme=theme,
                forbidden_topics=forbidden_topics
            )
            response = self.client_setup_story.call(req)
            self.get_logger().info(f"Parameters defined successfully: {response}")
        except Exception as e:
            self.get_logger().error(f"Setup story service call failed: {e}")

    def obtain_response(self):
        """Obtain response from the OpenAI assistant."""
        self.get_logger().info("Waiting for Obtain Response service...")
        self.client_obtain_response = self.create_client(ObtainResponse, 'obtain_response')
        while not self.client_obtain_response.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service obtain_response not available, waiting...')

        try:
            self.get_logger().info("Requesting a response from the assistant...")
            response = self.client_obtain_response.call(ObtainResponse.Request())
            self.queue_empty_event.wait()
            return response.output_text
        except Exception as e:
            self.get_logger().error(f"Obtain response service call failed: {e}")

    def shutdown_hook(self):
        """Clean up resources on shutdown."""
        self.get_logger().info("Shutting down StoryTelling node...")
        self.synthesize_queue.put(None)
        self.synthesize_thread.join()
        self.get_logger().info("StoryTelling node shut down successfully.")

def main(args=None):
    rclpy.init(args=args)
    storytelling = StoryTelling()

    try:
        rclpy.spin(storytelling)
    except KeyboardInterrupt:
        storytelling.shutdown_hook()
    finally:
        storytelling.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
