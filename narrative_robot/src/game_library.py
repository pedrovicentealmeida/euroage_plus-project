import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import queue
import threading
from std_msgs.msg import String

from speech_services.srv import TtsMicrosoft, Stt
from story_telling.srv import NewMessage, SetupStory, ObtainResponse


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

        # Subscription to the 'story_telling_text' topic
        self.create_subscription(String, 'story_telling_text', self.story_callback, 10)

        # Registering a shutdown hook
        self.add_on_shutdown(self.shutdown_hook)

    def synthesize_text(self):
        """Thread method to process text-to-speech requests."""
        self.get_logger().info("Waiting for TTS service...")
        self.wait_for_service('tts_microsoft')
        tts_client = self.create_client(TtsMicrosoft, 'tts_microsoft')

        while rclpy.ok():
            try:
                text = self.synthesize_queue.get(timeout=1)
                if text is None:
                    break

                request = TtsMicrosoft.Request()
                request.text = text
                request.language = "pt-PT"
                request.rate = "0"
                self.get_logger().info(f"Synthesizing speech: {text}")

                future = tts_client.call_async(request)
                rclpy.spin_until_future_complete(self, future)

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
        self.wait_for_service('stt_service')
        stt_client = self.create_client(Stt, 'stt_service')
        request = Stt.Request()
        request.language = "pt-PT"
        self.get_logger().info("Requesting speech-to-text...")

        future = stt_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def call_new_message_service(self, text):
        """Call the new_message service with the provided text."""
        self.wait_for_service('new_message')
        new_message_client = self.create_client(NewMessage, 'new_message')
        request = NewMessage.Request()
        request.input_text = text

        future = new_message_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"New message service response: {future.result()}")

    def define_parameters(self, name, age, brain, hobbies, profession, family, theme, forbidden_topics):
        """Define story parameters by calling the setup_story service."""
        self.wait_for_service('setup_story')
        setup_story_client = self.create_client(SetupStory, 'setup_story')
        request = SetupStory.Request(
            name=name,
            age=age,
            brain=brain,
            hobbies=hobbies,
            profession=profession,
            family=family,
            theme=theme,
            forbidden_topics=forbidden_topics
        )

        future = setup_story_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"Parameters defined successfully: {future.result()}")

    def obtain_response(self):
        """Obtain response from the OpenAI assistant."""
        self.wait_for_service('obtain_response')
        obtain_response_client = self.create_client(ObtainResponse, 'obtain_response')

        self.get_logger().info("Requesting a response from the assistant...")
        future = obtain_response_client.call_async(ObtainResponse.Request())
        rclpy.spin_until_future_complete(self, future)
        self.queue_empty_event.wait()
        return future.result().output_text

    def shutdown_hook(self):
        """Clean up resources on shutdown."""
        self.get_logger().info("Shutting down StoryTelling node...")
        self.synthesize_queue.put(None)
        self.synthesize_thread.join() 
        self.get_logger().info("StoryTelling node shut down successfully.")


def main(args=None):
    rclpy.init(args=args)
    storytelling = StoryTelling()

    executor = MultiThreadedExecutor()
    executor.add_node(storytelling)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        storytelling.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
