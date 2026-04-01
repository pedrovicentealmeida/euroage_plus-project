#!/usr/bin/env python3

##################################################################################
# BSD 3-Clause License
# 
# Copyright (c) 2025, Pedro V. Almeida
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
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

import rclpy
from rclpy.node import Node
import openai
from std_msgs.msg import String
from story_telling.srv import SetupStory, NewMessage, ObtainResponse


class StoryTelling:
    """Class to handle storytelling with OpenAI Chat Completions API."""

    def __init__(self, node, model):
        """Initialize the StoryTelling class."""
        self.node = node
        self.model = model
        self.client = openai.OpenAI(api_key="API_KEY_AQUI")
        self.publisher = node.create_publisher(String, 'story_telling_text', 10)

        # Conversation history (replaces Assistants API thread)
        self.messages = [
            {
                "role": "system",
                "content": (
                    "You are a storytelling assistant for elderly people with cognitive impairments. "
                    "Tell engaging, personalized stories based on the user's background and interests. "
                    "Keep sentences short and clear. Pause naturally at punctuation marks."
                )
            }
        ]

    def new_message(self, text: str) -> None:
        """Add a new user message to the conversation history."""
        self.messages.append({"role": "user", "content": text})

    def define_parameters(self, name, age, brain, hobbies, profession, family, theme, forbidden_topics) -> None:
        """Define story parameters based on user input."""
        info = (
            f"Jogador:\n {name} de {age} anos\n Gosta de {hobbies}\n"
            f" Nível de défice cognitivo {brain}\n Profissão passada: {profession}\n"
            f" Família/Amigos: {family}"
            f"História:\n Tema da história: {theme}\n Não fales em {forbidden_topics}"
        )
        self.new_message(info)

    def obtain_response(self) -> str:
        """Obtain streaming response from Chat Completions API, publishing sentence by sentence."""
        all_text = ""
        buffer = ""

        try:
            stream = self.client.chat.completions.create(
                model=self.model,
                messages=self.messages,
                stream=True,
            )

            for chunk in stream:
                delta = chunk.choices[0].delta
                if delta.content:
                    buffer += delta.content
                    all_text += delta.content

                    # Publish sentence by sentence
                    while True:
                        end_idx = -1
                        for punct in (".", "?", "!"):
                            idx = buffer.find(punct)
                            if idx != -1 and (end_idx == -1 or idx < end_idx):
                                end_idx = idx

                        if end_idx == -1:
                            break

                        sentence = buffer[:end_idx + 1]
                        buffer = buffer[end_idx + 1:]

                        msg = String()
                        msg.data = sentence.strip()
                        if msg.data:
                            self.publisher.publish(msg)

        except openai.OpenAIError as e:
            self.node.get_logger().error(f"OpenAI API error: {e}")
            return ""

        # Publish any remaining text that didn't end with punctuation
        if buffer.strip():
            msg = String()
            msg.data = buffer.strip()
            self.publisher.publish(msg)

        # Add assistant response to conversation history for context
        self.messages.append({"role": "assistant", "content": all_text})

        return all_text


class StoryTellingNode(Node):
    """ROS2 Node to handle storytelling services."""

    def __init__(self):
        super().__init__('story_telling_node')

        # Declare ROS2 parameter for the model (can be overridden at launch)
        self.declare_parameter('model', 'gpt-4o-mini')
        model = self.get_parameter('model').get_parameter_value().string_value

        self.get_logger().info(f"Using OpenAI model: {model}")
        self.st = StoryTelling(self, model)

        # Define services
        self.srv_setup_story = self.create_service(SetupStory, 'setup_story', self.handle_setup_story)
        self.srv_new_message = self.create_service(NewMessage, 'new_message', self.handle_new_message)
        self.srv_obtain_response = self.create_service(ObtainResponse, 'obtain_response', self.handle_obtain_response)

        self.get_logger().info("Story Telling services are ready.")

    def handle_setup_story(self, req, res):
        """Callback for the setup_story service."""
        self.st.define_parameters(
            req.name, req.age, req.brain, req.hobbies,
            req.profession, req.family, req.theme, req.forbidden_topics
        )
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