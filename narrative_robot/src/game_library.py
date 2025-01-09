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
import queue
import threading
from std_msgs.msg import String

from speech_services.srv import tts_microsoft, tts_microsoftRequest, stt, sttRequest
from story_telling.srv import setup_story, setup_storyRequest, new_message, new_messageRequest, obtain_response, obtain_responseRequest

class StoryTelling:
    """Class to handle storytelling with OpenAI assistant."""

    def __init__(self):
        """Initialize the StoryTelling class."""
        self.synthesize_queue = queue.Queue()
        self.queue_empty_event = threading.Event()
        self.synthesize_thread = threading.Thread(target=self.synthesize_text)
        self.synthesize_thread.daemon = True
        self.synthesize_thread.start()

        rospy.init_node('story_telling_interactive_node', anonymous=True)
        rospy.Subscriber('story_telling_text', String, self.story_callback)
        rospy.on_shutdown(self.shutdown_hook)
        #rospy.loginfo("Subscribed to 'story_telling_text' topic.")

    def synthesize_text(self):
        """Thread method to process text-to-speech requests."""
        rospy.wait_for_service('tts_microsoft')
        try:
            tts_client = rospy.ServiceProxy('tts_microsoft', tts_microsoft)
            while not rospy.is_shutdown():
                try:
                    text = self.synthesize_queue.get(timeout=1)
                    if text is None:
                        break

                    tts_req = tts_microsoftRequest(text=text, language="pt-PT", rate="0")
                    rospy.loginfo(f"Synthesizing speech: {text}")
                    tts_client(tts_req)
                    self.synthesize_queue.task_done()

                    if self.synthesize_queue.empty():
                        self.queue_empty_event.set()

                except queue.Empty:
                    continue

        except rospy.ServiceException as e:
            rospy.logerr(f"TTS service call failed: {e}")
        except Exception as e:
            rospy.logerr(f"Unexpected error in synthesize_text: {e}")

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
        rospy.wait_for_service('stt_service')
        try:
            stt_client = rospy.ServiceProxy('stt_service', stt)
            req = sttRequest(language="pt-PT")
            rospy.loginfo("Requesting speech-to-text...")
            return stt_client(req)
        except rospy.ServiceException as e:
            rospy.logerr(f"STT service call failed: {e}")
            return None

    def call_new_message_service(self, text):
        """Call the new_message service with the provided text."""
        rospy.wait_for_service('new_message')
        try:
            new_message_client = rospy.ServiceProxy('new_message', new_message)
            req = new_messageRequest(input_text=text)
            #rospy.loginfo(f"Sending new message: {text}")
            response = new_message_client(req)
            rospy.loginfo(f"New message service response: {response}")
        except rospy.ServiceException as e:
            rospy.logerr(f"New message service call failed: {e}")

    def define_parameters(self, name, age, brain, hobbies, profession, family, theme, forbidden_topics):
        """Define story parameters by calling the setup_story service."""
        rospy.wait_for_service('setup_story')
        try:
            setup_story_client = rospy.ServiceProxy('setup_story', setup_story)
            req = setup_storyRequest(
                name=name, age=age, brain=brain, hobbies=hobbies,
                profession=profession, family=family, theme=theme,
                forbidden_topics=forbidden_topics
            )
            response = setup_story_client(req)
            rospy.loginfo(f"Parameters defined successfully: {response}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Setup story service call failed: {e}")

    def obtain_response(self):
        """Obtain response from the OpenAI assistant."""
        rospy.wait_for_service('obtain_response')
        try:
            obtain_response_client = rospy.ServiceProxy('obtain_response', obtain_response)
            rospy.loginfo("Requesting a response from the assistant...")
            response = obtain_response_client(obtain_responseRequest())
            self.queue_empty_event.wait()
            return response.output_text
            #rospy.loginfo(f"Response from assistant: {response.output_text}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Obtain response service call failed: {e}")

    def shutdown_hook(self):
        """Clean up resources on shutdown."""
        rospy.loginfo("Shutting down StoryTelling node...")
        self.synthesize_queue.put(None)
        self.synthesize_thread.join() 
        rospy.loginfo("StoryTelling node shut down successfully.")

if __name__ == "__main__":
    try:
        storytelling = StoryTelling()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
