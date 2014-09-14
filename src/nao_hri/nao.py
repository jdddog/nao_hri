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

from hri_api.entities import Robot
from hri_api.entities import Expression, Gesture
import actionlib
from nao_msgs.msg import BlinkAction, BlinkGoal
from std_msgs.msg import ColorRGBA

class NaoExpression(Expression):
    def __init__(*args):
        Expression.__init__(*args)

    red_eyes = 1
    red_leye = 2
    red_reye = 3

    blue_eyes = 4
    blue_leye = 5
    blue_reye = 6

    green_eyes = 7
    green_leye = 8
    green_reye = 9

    wink_leye = 10
    wink_reye = 11


class NaoGesture(Gesture):
    def __init__(*args):
        Gesture.__init__(*args)

    # Head
    nod_head = 5
    shake_head = 6

    # Arms
    wave_larm = 1
    wave_rarm = 2

    point_larm = 3
    point_rarm = 4

    hands_on_hips = 7
    l_hand_on_lhip = 7
    r_hand_on_rhip = 8


class Nao(Robot):

    def __init__(self):
        Robot.__init__(self, NaoExpression, NaoGesture)
        self.blink_client = actionlib.SimpleActionClient('blink', BlinkAction)
        self.wait_for_action_servers(self.blink_client)

    def blink(self, blink_duration, blink_rate_mean, blink_rate_sd):
        goal = BlinkGoal()
        goal.blink_duration = blink_duration
        goal.blink_rate_mean = blink_rate_mean
        goal.blink_rate_sd = blink_rate_sd
        goal.bg_color = ColorRGBA(r=0, g=0, b=255, a=10)
        goal.colors = [ColorRGBA(r=0, g=0, b=255, a=255),
                       ColorRGBA(r=0, g=0, b=255, a=200),
                       ColorRGBA(r=0, g=0, b=255, a=220)]

        self.blink_client.send_goal(goal)






