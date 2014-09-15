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



from hri_api.entities import Person, World, Saliency
from hri_api.query import Query
from nao_hri.nao import Nao, NaoExpression, NaoGesture
import random

# Initialize World, Nao and People (their coordinates are specified in the launch file)
world = World()
robot = Nao()   # Initialize Nao robot
people = [Person(1), Person(2), Person(3)]
utterances = ["Why are you looking at me?",
              "Are you pregnant?",
              "What's going on in that head of yours?",
              "Do you think I'm rude?"]

# Make nao gaze at each person
for person in people:
    robot.gaze_and_wait(person.head, speed=0.5)

# Change Nao's eye color
# robot.expression(NaoExpression.red_leye)
# robot.expression(NaoExpression.green_reye)

# Make Nao speak and gesture to audience of people
robot.say_to("Hello my name is Nao!", people)

# Say random things and point at peoples heads
for person in people:
    text = "<{point} target={target}> {utterance} <{point}/>".format(point=NaoGesture.point_larm,
                                                                     utterance=random.choice(utterances),
                                                                     target=person.head)
    robot.say_to_and_wait(text, person)
    robot.wait_for_period(5.0)

robot.say_to("Goodbye!", people)