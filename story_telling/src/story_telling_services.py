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
import openai

from openai import AssistantEventHandler
from typing_extensions import override
from std_msgs.msg import String

from story_telling.srv import setup_story, setup_storyResponse
from story_telling.srv import new_message, new_messageResponse
from story_telling.srv import obtain_response, obtain_responseResponse

st = None

class EventHandler(AssistantEventHandler):
    """Custom event handler for OpenAI assistant events."""
    
    def __init__(self):
        """Initialize the event handler."""
        super().__init__()
        self.text_final = "" 
        self.all_text = ""
        self.control = False
        self.publisher = rospy.Publisher('story_telling_text', String, queue_size=10)

    @override
    def on_text_delta(self, delta, snapshot):
        """Callback function triggered on receiving text delta."""
        self.text_final += delta.value
        #print(delta.value, end="", flush=True)
        
        # Frase a frase
        if (delta.value in (".", "?", "!")): 
            self.publisher.publish(self.text_final)
            self.all_text += self.text_final
            self.text_final = ""

class StoryTelling:
    """Class to handle storytelling with OpenAI assistant."""

    def __init__(self):
        """Initialize the StoryTelling class."""
        self.client = openai.OpenAI(api_key="YOUR-API-KEY")
        self.thread = self.client.beta.threads.create()

    def new_message(self, text: str) -> None:
        """Send a new message to the OpenAI assistant."""
        try:
            message = self.client.beta.threads.messages.create(
                thread_id=self.thread.id,
                role="user",
                content=text
            )
        except openai.error.OpenAIError as e:
            rospy.logerr(f"Failed to send message: {e}")
            raise e


    def define_parameters(self, name, age, brain, hobbies, profession, family, theme, forbidden_topics) -> None:
        """Define story parameters based on user input."""
        info = (f"Jogador:\n {name} de {age} anos\n Gosta de {hobbies}\n Nível de défice cognitivo {brain}\n Profissão passada: {profession}\n Família/Amigos: {family}"
                f"História:\n Tema da história: {theme}\n Não fales em {forbidden_topics}")

        self.new_message(info)

    def obtain_response(self) -> str:
        """Obtain response from the OpenAI assistant."""
        event_handler = EventHandler()
        with self.client.beta.threads.runs.create_and_stream(
            thread_id=self.thread.id,
            assistant_id="YOUR-ASSISTANT-ID",
            event_handler=event_handler,
        ) as stream:
            stream.until_done()
        
        event_handler.all_text += event_handler.text_final
        return event_handler.all_text

def handle_setup_story(req):
    """Callback for the setup_story service."""
    st.define_parameters(req.name, req.age, req.brain, req.hobbies, req.profession, req.family, req.theme, req.forbidden_topics)
    return setup_storyResponse(success=True)

def handle_new_message(req):
    """Callback for the new_message service."""
    st.new_message(req.input_text)
    return new_messageResponse(success=True)

def handle_obtain_response(req):
    """Callback for the obtain_response service."""
    response_text = st.obtain_response()
    return obtain_responseResponse(output_text=response_text)

def main():
    
    global st
    
    rospy.init_node('story_telling_node')
    
    st = StoryTelling()
    
    rospy.Service('setup_story', setup_story, handle_setup_story)
    rospy.Service('new_message', new_message, handle_new_message)
    rospy.Service('obtain_response', obtain_response, handle_obtain_response)
    
    rospy.loginfo("Story Telling services ready to use.")
    rospy.spin()

if __name__ == "__main__":
    main()
