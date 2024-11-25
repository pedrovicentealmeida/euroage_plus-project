#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import openai
from std_msgs.msg import String
from story_telling.srv import SetupStory, NewMessage, ObtainResponse

class EventHandler(openai.AssistantEventHandler):
    """Custom event handler for OpenAI assistant events."""

    def __init__(self, node):
        """Initialize the event handler with a ROS2 node."""
        super().__init__()
        self.text_final = ""
        self.all_text = ""
        self.control = False
        self.publisher = node.create_publisher(String, 'story_telling_text', 10)

    def on_text_delta(self, delta, snapshot):
        """Callback function triggered on receiving text delta."""
        self.text_final += delta.value

        # Publish sentence by sentence
        if delta.value in (".", "?", "!"):
            msg = String()
            msg.data = self.text_final
            self.publisher.publish(msg)
            self.all_text += self.text_final
            self.text_final = ""


class StoryTelling:
    """Class to handle storytelling with OpenAI assistant."""

    def __init__(self, node):
        """Initialize the StoryTelling class."""
        self.node = node  # Save the node reference
        self.client = openai.OpenAI(api_key="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")
        self.thread = self.client.beta.threads.create()

    def new_message(self, text: str) -> None:
        """Send a new message to the OpenAI assistant."""
        try:
            self.client.beta.threads.messages.create(
                thread_id=self.thread.id,
                role="user",
                content=text
            )
        except openai.error.OpenAIError as e:
            raise e

    def define_parameters(self, name, age, brain, hobbies, profession, family, theme, forbidden_topics) -> None:
        """Define story parameters based on user input."""
        info = (f"Jogador:\n {name} de {age} anos\n Gosta de {hobbies}\n Nível de défice cognitivo {brain}\n Profissão passada: {profession}\n Família/Amigos: {family}"
                f"História:\n Tema da história: {theme}\n Não fales em {forbidden_topics}")
        
        self.new_message(info)

    def obtain_response(self) -> str:
        """Obtain response from the OpenAI assistant."""
        event_handler = EventHandler(self.node)  # Pass the node here
        with self.client.beta.threads.runs.stream(
            thread_id=self.thread.id,
            assistant_id="asst_SnlrOiiPlAvoLztLBsMGRNiO",
            event_handler=event_handler,
        ) as stream:
            stream.until_done()

        event_handler.all_text += event_handler.text_final
        return event_handler.all_text

class StoryTellingNode(Node):
    """ROS2 Node to handle storytelling services."""

    def __init__(self):
        super().__init__('story_telling_node')

        self.st = StoryTelling(self)  # Pass the node to StoryTelling

        # Define services
        self.srv_setup_story = self.create_service(SetupStory, 'setup_story', self.handle_setup_story)
        self.srv_new_message = self.create_service(NewMessage, 'new_message', self.handle_new_message)
        self.srv_obtain_response = self.create_service(ObtainResponse, 'obtain_response', self.handle_obtain_response)

        self.get_logger().info("Story Telling services are ready.")

    def handle_setup_story(self, req, res):
        """Callback for the setup_story service."""
        self.st.define_parameters(req.name, req.age, req.brain, req.hobbies, req.profession, req.family, req.theme, req.forbidden_topics)
        res.success = True
        return res

    def handle_new_message(self, req, res):
        """Callback for the new_message service."""
        self.st.new_message(req.input_text)
        res.success = True
        return res

    def handle_obtain_response(self, req, res):
        """Callback for the obtain_response service."""
        response_text = self.st.obtain_response()
        res.output_text = response_text
        return res


def main(args=None):
    rclpy.init(args=args)
    node = StoryTellingNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
