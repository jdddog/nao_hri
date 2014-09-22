#!/usr/bin/env python
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


from hri_api.entities import World, Person
from nao_hri import Nao, Gesture
from hri_api.query import Query


world = World()
robot = Nao()
people = Query([Person(1), Person(2), Person(3)])


#TODO: allow lists to be passed

robot.say_to_and_wait('hello who are you?', people)
#robot.say_to_and_wait('Its hazards are hostile to us all. '
                      # 'Its conquest deserves the best of all '
                      # 'mankind, and its opportunity for peaceful '
                      # 'cooperation may never come again. But why, some say, the moon? '
                      # 'Why choose this as our goal? And they may well ask why climb the highest mountain? '
                      # 'Why, 35 years ago, fly the Atlantic?', people)

person = people.execute()[0]
robot.say_to_and_wait("<PointLarm target='{0}'> no its not your turn David Cunliffe! </PointLarm>".format(person.head), person)


