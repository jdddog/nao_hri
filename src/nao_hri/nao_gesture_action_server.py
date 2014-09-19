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
from hri_framework import IGestureActionServer
from nao_hri import NaoNode
from nao_hri import NaoGesture
from threading import Timer


class GestureHandle():
    def __init__(self, goal_handle, motion_id, timer):
        self.goal_id = goal_handle.get_goal_id().id
        self.goal_handle = goal_handle
        self.motion_id = motion_id
        self.timer = timer


class NaoGestureActionServer(IGestureActionServer, NaoNode):

    def __init__(self):
        IGestureActionServer.__init__(self, NaoGesture)
        self.gesture_handle_lookup = {}
        self.motion_proxy = None

    def start(self):
        NaoNode.__init__(self, self.get_instance_name())
        self.motion_proxy = self.get_proxy('ALMotion')
        super(NaoGestureActionServer, self).start()

    def start_gesture(self, goal_handle):
        goal = goal_handle.get_goal()

        if self.has_gesture(goal.gesture):
            gesture = NaoGesture[goal.gesture]

            animations = NaoGesture.get_keyframe_animations(gesture)
            names = []
            times = []
            keys = []

            for a in animations:
                (n_temp, t_temp, k_temp) = a.get_ntk(goal.duration)
                names += n_temp
                times += t_temp
                keys += k_temp

            motion_id = self.motion_proxy.post.angleInterpolationBezier(names, times, keys)
            timer = Timer(goal.duration, self.gesture_finished, [goal_handle])
            gesture_handle = GestureHandle(goal_handle, motion_id, timer)
            self.add_gesture_handle(gesture_handle)
            timer.start()

        else:
            self.action_server.set_aborted(goal_handle)

    def cancel_gesture(self, goal_handle):
        gesture_handle = self.get_gesture_handle(goal_handle)
        gesture_handle.timer.cancel()
        self.motion_proxy.stop(gesture_handle.motion_id)
        self.remove_gesture_handle(goal_handle)

    def gesture_finished(self, goal_handle):
        super(NaoGestureActionServer, self).gesture_finished(goal_handle)
        self.remove_gesture_handle(goal_handle)

    def get_gesture_handle(self, goal_handle):
        return self.gesture_handle_lookup[goal_handle.get_goal_id().id]

    def add_gesture_handle(self, gesture_handle):
        self.gesture_handle_lookup[gesture_handle.goal_id] = gesture_handle

    def remove_gesture_handle(self, goal_handle):
        self.gesture_handle_lookup.pop(goal_handle.get_goal_id().id)

if __name__ == '__main__':
    rospy.init_node('gesture_action_server')
    gesture_server = NaoGestureActionServer()
    gesture_server.start()
    rospy.spin()
