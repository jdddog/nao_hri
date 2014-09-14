# Copyright (c) 2014, James Diprose
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
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

import rospy
import re
import md5
from hri_framework.srv import TextToSpeechSubsentenceDurationResponse
from hri_framework import TextToSpeechActionServer
from nao_hri import NaoNode
import rospkg


class TextToSpeechCache():
    def __init__(self):
        self.tts_time_hashes = {}
        self.file_name = None

    def load_tts_cache(self, path):
        self.file_name = path
        self.tts_time_hashes = {}

        tts_cache = open(path, "r")

        for line in tts_cache:
            columns = re.split(";", line.strip())
            if len(columns) == 3:
                md5_hash = columns[0]
                times = re.split(",", columns[1])
                self.tts_time_hashes[md5_hash] = times

        tts_cache.close()

        rospy.loginfo("TTS cache loaded. File: {0} ".format(path))

    def get_id(self, sentence):
        return str(md5.new(sentence).hexdigest())

    def is_cached(self, sentence):
        md5_digest = self.get_id(sentence)

        if md5_digest in self.tts_time_hashes:
            return True
        else:
            return False

    def get_sentence_times(self, sentence):
        return self.tts_time_hashes[self.get_id(sentence)]

    def add_sentence(self, sentence, word_times):
        with open(self.file_name, "a") as tts_cache:
            md5_digest = self.get_id(sentence)
            tts_cache.write(md5_digest + ";" + str(word_times).replace("[", "").replace("]", "") + ";" + sentence.replace(";", "") + "\n")
            self.tts_time_hashes[md5_digest] = word_times


class NaoTextToSpeechActionServer(TextToSpeechActionServer):

    def __init__(self):
        TextToSpeechActionServer.__init__(self)
        self.tts_cache = TextToSpeechCache()

        # Text to speech arguments
        self.num_words_sentence = 0
        self.current_word_index = 0
        self.word_times = []
        self.start_time = None
        self.calc_synth_times = True
        self.utterance_id = None

        # Naoqi
        self.tts_proxy = None
        self.mem_proxy = None
        self.nao_node = NaoNode()

    def start(self):
        if rospy.has_param("nao_tts_cache"):
            path = rospy.has_param("nao_tts_cache")
            self.tts_cache.load_tts_cache(path)
            self.tts_proxy = self.nao_node.get_proxy('ALTextToSpeech')
            self.mem_proxy = self.nao_node.get_proxy('ALMemory')
            TextToSpeechActionServer.start_server(self)
        else:
            raise Exception('nao_tts_cache parameter not found')

    def synthesise_sentence(self, sentence):
        self.num_words_sentence = NaoTextToSpeechActionServer.words_in_text(sentence)
        self.current_word_index = 0
        self.word_times = []
        self.word_times.append(0.0)
        self.start_time = rospy.get_time()

        if self.tts_cache.is_cached(sentence):
            self.calc_synth_times = False
        else:
            self.calc_synth_times = True

        self.utterance_id = self.tts_proxy.post.say(sentence)
        rospy.loginfo("Synthesis started: '{0}'".format(sentence))

    def tts_subsentence_duration(self, sentence, start_word_index, end_word_index):
        # If have spoken sentence before, then get the length of time it took to speak each word
        # and return duration of subsentence
        if self.tts_cache.is_cached(sentence):
            times = self.tts_cache.get_sentence_times(sentence)
            duration = float(times[end_word_index]) - float(times[start_word_index])  # End word is beginning of next word
        else:
            # Else estimate the length of time it takes to speak a subsentence
            words = NaoTextToSpeechActionServer.get_words(sentence)
            sub_text = ''
            for i in range(start_word_index, end_word_index):
                sub_text += words[i] + ' '

            num_characters = len(sub_text)
            duration = num_characters * 0.092055994

        return duration

    def current_word_changed(self, event_name, value, sub_id):

        if self.current_word_index != 0 and self.server.is_active():
            word_start_time = rospy.get_time() - self.start_time
            self.word_times.append(word_start_time)

        self.send_feedback(self.current_word_index)
        self.current_word_index += 1

    def word_pos_changed(self, event_name, value, sub_id):

        # If at end of sentence...
        if self.server.is_active() and self.current_word_index >= self.num_words_sentence and value == 0:
            word_start_time = rospy.get_time() - self.start_time
            self.word_times.append(word_start_time)

            if self.calc_synth_times:
                self.tts_cache.add_sentence(self.server.current_goal.get_goal().sentence, self.word_times)

            self.synthesis_finished()

    def on_shutdown(self):
        self.unsubscribe()

    def cancel_tts_synthesis(self):
        self.tts_proxy.stop(self.utterance_id)

    @staticmethod
    def words_in_text(text):
        valid_words_regex = "\w+[']{0,1}\w*[!?,.]{0,1}"
        return len(re.findall(valid_words_regex, text))

    @staticmethod
    def get_words(text):
        valid_words_regex = "\w+[']{0,1}\w*[!?,.]{0,1}"
        return re.findall(valid_words_regex, text)

    # A stupid hack:
    #   Instead of taking a function pointer as a callback (e.g. self.word_pos_changed)
    #   subscribeToEvent / unsubscribeToEvent take the name of the class instance as a string and the name of the method
    #   as a string.
    #
    #   Hence, this method finds it for you automatically

    def get_instance_name(self):
        instance_name = ''

        for k, v in list(locals().iteritems()):
            if v is self:
                instance_name = k

        return instance_name

    def subscribe(self):
        instance_name = self.get_instance_name()
        self.tts_proxy.enableNotifications()
        self.mem_proxy.subscribeToEvent("ALTextToSpeech/PositionOfCurrentWord", instance_name, "word_pos_changed")
        self.mem_proxy.subscribeToEvent("ALTextToSpeech/CurrentWord", instance_name, "current_word_changed")
        rospy.loginfo("Subscribed to ALTextToSpeech events.")

    def unsubscribe(self):
        instance_name = self.get_instance_name()
        self.mem_proxy.unsubscribeToEvent("ALTextToSpeech/PositionOfCurrentWord", instance_name)
        self.mem_proxy.unsubscribeToEvent("ALTextToSpeech/CurrentWord", instance_name)
        self.tts_proxy.disableNotifications()
        rospy.loginfo("Un-subscribed from ALTextToSpeech events.")

