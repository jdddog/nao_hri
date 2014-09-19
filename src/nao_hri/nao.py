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
from hri_api.entities import IExpression, IGesture
import actionlib
from nao_msgs.msg import BlinkAction, BlinkGoal
from std_msgs.msg import ColorRGBA
from nao_hri import AtomicAnimation, RepeatingAnimation


class Expression(IExpression):
    def __init__(*args):
        IExpression.__init__(*args)

    RedEyes = 1
    RedLeye = 2
    RedReye = 3

    BlueEyes = 4
    BlueLeye = 5
    BlueReye = 6

    GreenEyes = 7
    GreenLeye = 8
    GreenReye = 9

    WinkLeye = 10
    WinkReye = 11

    def default_speed(self):
        expression_data = self.get_data()
        return expression_data[1]

    def default_intensity(self):
        expression_data = self.get_data()
        return expression_data[2]

    def default_duration(self):
        expression_data = self.get_data()
        return expression_data[3]


class Gesture(IGesture):
    def __init__(self, *args):
        IGesture.__init__(*args)

        self.data = [(Gesture.LarmDown, 2.0),
            (Gesture.RarmDown, 2.0),
            (Gesture.WaveLarm, 6.0),
            (Gesture.MotionRight, 2.0),
            (Gesture.MotionLeft, 2.0),
            (Gesture.HandsOnHips, 3.0),
            (Gesture.PointLarm, None),
            (Gesture.PointRarm, None)]

    LarmDown = 1
    RarmDown = 2
    WaveLarm = 3
    MotionRight = 4
    MotionLeft = 5
    HandsOnHips = 6
    PointLarm = 7
    PointRarm = 8

    def default_duration(self):
        gesture_data = self.get_data()
        return gesture_data[1]

    def get_keyframe_animations(self):
        names = []
        times = []
        keys = []
        animations = []

        if self is Gesture.LarmDown:
            names.append("LElbowRoll")
            times.append([ 0.80000])
            keys.append([ [ -0.42242, [ 3, -0.26667, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("LElbowYaw")
            times.append([ 0.80000])
            keys.append([ [ -1.20310, [ 3, -0.26667, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("LHand")
            times.append([ 0.80000])
            keys.append([ [ 0.30000, [ 3, -0.26667, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("LShoulderPitch")
            times.append([ 0.80000])
            keys.append([ [ 1.45089, [ 3, -0.26667, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("LShoulderRoll")
            times.append([ 0.80000])
            keys.append([ [ 0.13632, [ 3, -0.26667, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("LWristYaw")
            times.append([ 0.90000])
            keys.append([ [ 0.11491, [ 3, -0.26667, 0.00000], [ 3, 0.00000, 0.00000]]])

            animations.append(AtomicAnimation(names, times, keys))

        elif self is Gesture.RarmDown:
            names.append("RElbowRoll")
            times.append([ 0.80000])
            keys.append([ [ 0.42506, [ 3, -0.26667, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("RElbowYaw")
            times.append([ 0.80000])
            keys.append([ [ 1.17706, [ 3, -0.26667, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("RHand")
            times.append([ 0.80000])
            keys.append([ [ 0.30000, [ 3, -0.26667, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("RShoulderPitch")
            times.append([ 0.80000])
            keys.append([ [ 1.44378, [ 3, -0.26667, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("RShoulderRoll")
            times.append([ 0.80000])
            keys.append([ [ -0.12589, [ 3, -0.26667, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("RWristYaw")
            times.append([ 0.80000])
            keys.append([ [ 0.09794, [ 3, -0.26667, 0.00000], [ 3, 0.00000, 0.00000]]])

            animations.append(AtomicAnimation(names, times, keys))

        elif self is Gesture.WaveLarm:
            names.append("LElbowRoll")
            times.append([ 0.60000, 1.20000])
            keys.append([ [ -1.36982, [ 3, -0.20000, 0.00000], [ 3, 0.20000, 0.00000]], [ -1.54930, [ 3, -0.20000, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("LElbowYaw")
            times.append([ 0.60000, 1.20000])
            keys.append([ [ -2.07555, [ 3, -0.20000, 0.00000], [ 3, 0.20000, 0.00000]], [ -1.03703, [ 3, -0.20000, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("LShoulderPitch")
            times.append([ 0.60000, 1.20000])
            keys.append([ [ 0.23926, [ 3, -0.20000, 0.00000], [ 3, 0.20000, 0.00000]], [ 0.14262, [ 3, -0.20000, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("LShoulderRoll")
            times.append([ 0.60000, 1.20000])
            keys.append([ [ 0.39880, [ 3, -0.20000, 0.00000], [ 3, 0.20000, 0.00000]], [ 0.19324, [ 3, -0.20000, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("LWristYaw")
            times.append([ 0.60000, 1.20000])
            keys.append([ [ 1.39283, [ 3, -0.20000, 0.00000], [ 3, 0.20000, 0.00000]], [ 1.06455, [ 3, -0.20000, 0.00000], [ 3, 0.00000, 0.00000]]])

            oscilate_arm = RepeatingAnimation(names, times, keys, 1)

            names = []
            times = []
            keys = []
            open_finger_duration = 1.2

            names.append("LHand")
            times.append([ 0.60000, 1.20000])
            keys.append([ [ 1.0, [ 3, -0.20000, 0.00000], [ 3, 0.20000, 0.00000]], [ 1.0, [ 3, -0.20000, 0.00000], [ 3, 0.00000, 0.00000]]])

            open_fingers = AtomicAnimation(names, times, keys, override_time=open_finger_duration)

            animations.append(oscilate_arm)
            animations.append(open_fingers)

        elif self is Gesture.MotionRight:

            names.append("RElbowRoll")
            times.append([ 0.64000])
            keys.append([ [ 0.42649, [ 3, -0.21333, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("RElbowYaw")
            times.append([ 0.64000])
            keys.append([ [ 1.31460, [ 3, -0.21333, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("RHand")
            times.append([ 0.64000])
            keys.append([ [ 1.0, [ 3, -0.21333, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("RShoulderPitch")
            times.append([ 0.64000])
            keys.append([ [ 0.07061, [ 3, -0.21333, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("RShoulderRoll")
            times.append([ 0.64000])
            keys.append([ [ -1.19043, [ 3, -0.21333, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("RWristYaw")
            times.append([ 0.64000])
            keys.append([ [ -0.61057, [ 3, -0.21333, 0.00000], [ 3, 0.00000, 0.00000]]])

            animations.append(AtomicAnimation(names, times, keys))

        elif self is Gesture.MotionLeft:
            names.append("LElbowRoll")
            times.append([ 1.24000])
            keys.append([ [ -0.49544, [ 3, -0.41333, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("LElbowYaw")
            times.append([ 1.24000])
            keys.append([ [ -0.19333, [ 3, -0.41333, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("LHand")
            times.append([ 1.24000])
            keys.append([ [ 0.97230, [ 3, -0.41333, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("LShoulderPitch")
            times.append([ 1.24000])
            keys.append([ [ -0.69494, [ 3, -0.41333, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("LShoulderRoll")
            times.append([ 1.24000])
            keys.append([ [ 1.20108, [ 3, -0.41333, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("LWristYaw")
            times.append([ 1.24000])
            keys.append([ [ 0.10887, [ 3, -0.41333, 0.00000], [ 3, 0.00000, 0.00000]]])

            animations.append(AtomicAnimation(names, times, keys))

        elif self is Gesture.HandsOnHips:
            names.append("LElbowRoll")
            times.append([ 1.00000, 2.08000])
            keys.append([ [ -0.74702, [ 3, -0.33333, 0.00000], [ 3, 0.36000, 0.00000]], [ -1.49101, [ 3, -0.36000, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("LElbowYaw")
            times.append([ 1.00000, 2.08000])
            keys.append([ [ -0.43570, [ 3, -0.33333, 0.00000], [ 3, 0.36000, 0.00000]], [ -0.27616, [ 3, -0.36000, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("LHand")
            times.append([ 1.00000, 2.08000])
            keys.append([ [ 0.00528, [ 3, -0.33333, 0.00000], [ 3, 0.36000, 0.00000]], [ 0.00004, [ 3, -0.36000, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("LShoulderPitch")
            times.append([ 1.00000, 2.08000])
            keys.append([ [ 1.55083, [ 3, -0.33333, 0.00000], [ 3, 0.36000, 0.00000]], [ 1.63980, [ 3, -0.36000, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("LShoulderRoll")
            times.append([ 1.00000, 2.08000])
            keys.append([ [ 0.53532, [ 3, -0.33333, 0.00000], [ 3, 0.36000, 0.00000]], [ 0.84059, [ 3, -0.36000, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("LWristYaw")
            times.append([ 1.00000, 2.08000])
            keys.append([ [ 0.13955, [ 3, -0.33333, 0.00000], [ 3, 0.36000, 0.00000]], [ 0.13955, [ 3, -0.36000, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("RElbowRoll")
            times.append([ 1.00000, 2.08000])
            keys.append([ [ 0.69494, [ 3, -0.33333, 0.00000], [ 3, 0.36000, 0.00000]], [ 1.54171, [ 3, -0.36000, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("RElbowYaw")
            times.append([ 1.00000, 2.08000])
            keys.append([ [ 0.43101, [ 3, -0.33333, 0.00000], [ 3, 0.36000, 0.00000]], [ 0.23313, [ 3, -0.36000, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("RHand")
            times.append([ 1.00000, 2.08000])
            keys.append([ [ 0.00533, [ 3, -0.33333, 0.00000], [ 3, 0.36000, 0.00000]], [ 0.00037, [ 3, -0.36000, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("RShoulderPitch")
            times.append([ 1.00000, 2.08000])
            keys.append([ [ 1.60461, [ 3, -0.33333, 0.00000], [ 3, 0.36000, 0.00000]], [ 1.61074, [ 3, -0.36000, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("RShoulderRoll")
            times.append([ 1.00000, 2.08000])
            keys.append([ [ -0.58450, [ 3, -0.33333, 0.00000], [ 3, 0.36000, 0.00000]], [ -0.84681, [ 3, -0.36000, 0.00000], [ 3, 0.00000, 0.00000]]])

            names.append("RWristYaw")
            times.append([ 1.00000, 2.08000])
            keys.append([ [ 0.05518, [ 3, -0.33333, 0.00000], [ 3, 0.36000, 0.00000]], [ 0.03217, [ 3, -0.36000, 0.00000], [ 3, 0.00000, 0.00000]]])

            animations.append(AtomicAnimation(names, times, keys))

        return animations


class Nao(Robot):

    def __init__(self):
        Robot.__init__(self, Expression, Gesture)
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






