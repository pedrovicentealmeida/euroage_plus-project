import rospy
import openai

from openai import AssistantEventHandler
from typing_extensions import override
from story_telling.srv import setup_story, setup_storyResponse
from story_telling.srv import generate_response, generate_responseResponse

st = None

class EventHandler(AssistantEventHandler):
    """Custom event handler for OpenAI assistant events."""
    
    def __init__(self):
        """Initialize the event handler."""
        super().__init__()
        self.text_final = "" 
        self.all_text = ""
        self.control = False

    @override
    def on_text_delta(self, delta, snapshot):
        """Callback function triggered on receiving text delta."""
        self.text_final += delta.value
        print(delta.value, end="", flush=True)

class StoryTelling:
    """Class to handle storytelling with OpenAI assistant."""

    def __init__(self):
        """Initialize the StoryTelling class."""
        self.client = openai.OpenAI(api_key="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")
        self.thread = self.client.beta.threads.create()

    def new_message(self, text: str) -> None:
        """Send a new message to the OpenAI assistant."""
        message = self.client.beta.threads.messages.create(
            thread_id=self.thread.id,
            role="user",
            content=text
        )

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
            assistant_id="asst_SnlrOiiPlAvoLztLBsMGRNiO",
            event_handler=event_handler,
        ) as stream:
            stream.until_done()
        
        while True:
            if not event_handler.control:
                break

        event_handler.all_text += event_handler.text_final
        return event_handler.all_text

def handle_setup_story(req):
    """Callback for the setup_story service."""
    st.define_parameters(req.name, req.age, req.brain, req.hobbies, req.profession, req.family, req.theme, req.forbidden_topics)
    initial_response = st.obtain_response()
    
    return setup_storyResponse(response=initial_response)

def handle_generate_response(req):
    """Callback for the generate_response service."""
    st.new_message(req.input_text)
    response_text = st.obtain_response()
    
    return generate_responseResponse(output_text=response_text)

def main():
    
    global st
    
    rospy.init_node('story_telling_node')
    
    st = StoryTelling()
    
    rospy.Service('setup_story', setup_story, handle_setup_story)
    rospy.Service('generate_response', generate_response, handle_generate_response)
    
    rospy.loginfo("Story Telling services ready to use.")
    rospy.spin()

if __name__ == "__main__":
    main()