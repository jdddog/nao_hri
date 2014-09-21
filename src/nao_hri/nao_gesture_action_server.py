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

import rospy
import actionlib
from hri_framework import IGestureActionServer
from nao_hri import NaoNode, Gesture
from hri_msgs.msg import TargetAction, TargetGoal
from threading import Timer, RLock


class GestureHandle():
    def __init__(self, goal_handle, gesture, motion_id=None, client=None):
        self.goal_id = goal_handle.get_goal_id().id
        self.gesture = gesture
        self.goal_handle = goal_handle
        self.timer = None
        self.motion_id = motion_id
        self.client = client

    def start_timer(self, duration, callback, *args):
        self.timer = Timer(duration, callback, *args)
        self.timer.start()

    def stop_timer(self):
        if self.timer is not None:
            self.timer.cancel()

class NaoGestureActionServer(IGestureActionServer, NaoNode):

    def __init__(self):
        IGestureActionServer.__init__(self, Gesture)
        self.motion_proxy = None
        self.lock = RLock()

        self.larm_client = actionlib.SimpleActionClient('nao_point_left', TargetAction)
        self.larm_gh = None

        self.rarm_client = actionlib.SimpleActionClient('nao_point_right', TargetAction)
        self.rarm_gh = None

    def start(self):
        NaoNode.__init__(self, self.get_instance_name())
        self.motion_proxy = self.get_proxy('ALMotion')
        super(NaoGestureActionServer, self).start()

    def is_keyframe_animation(self, gesture):
        if gesture in [Gesture.LarmDown, Gesture.RarmDown, Gesture.MotionRight, Gesture.MotionLeft, Gesture.WaveLarm]:
            return True
        return False

    def start_gesture(self, goal_handle):
        with self.lock:
            goal = goal_handle.get_goal()

            if self.is_valid_gesture(goal.gesture):
                gesture = Gesture[goal.gesture]

                if goal.duration is None:
                    duration = gesture.default_duration()
                else:
                    duration = goal.duration

                if self.is_keyframe_animation(gesture):
                    animations = gesture.keyframe_animations()
                    names = []
                    times = []
                    keys = []

                    for a in animations:
                        (n_temp, t_temp, k_temp) = a.get_ntk(duration)
                        names += n_temp
                        times += t_temp
                        keys += k_temp

                    motion_id = self.motion_proxy.post.angleInterpolationBezier(names, times, keys)
                    gesture_handle = GestureHandle(goal_handle, gesture, motion_id=motion_id)
                    self.add_gesture_handle(gesture_handle)
                    gesture_handle.start_timer(duration, self.set_succeeded, goal_handle)

                else:
                    target_goal = TargetGoal()
                    target_goal.target = goal.target
                    target_goal.speed = 0.5
                    target_goal.acceleration = 0.3

                    if gesture is Gesture.PointLarm:
                        self.larm_gh = goal_handle
                        client = self.larm_client
                        done_cb = self.larm_succeeded
                    elif gesture is Gesture.PointRarm:
                        self.rarm_gh = goal_handle
                        client = self.rarm_client
                        done_cb = self.rarm_succeeded

                    gesture_handle = GestureHandle(goal_handle, gesture, client=client)
                    self.add_gesture_handle(gesture_handle)

                    if goal.duration is None:
                        client.send_goal(target_goal, done_cb=done_cb)
                    else:
                        client.send_goal(target_goal)
                        gesture_handle.start_timer(duration, self.set_succeeded, goal_handle)
            else:
                self.set_aborted(goal_handle)

    def larm_succeeded(self):
        with self.lock:
            gesture_handle = self.get_gesture_handle(self.larm_gh)
            self.set_succeeded(self.larm_gh)
            self.larm_gh = None
            self.remove_gesture_handle(gesture_handle)

    def rarm_succeeded(self):
        with self.lock:
            gesture_handle = self.get_gesture_handle(self.rarm_gh)
            self.set_succeeded(self.rarm_gh)
            self.rarm_gh = None
            self.remove_gesture_handle(gesture_handle)

    def larm_cancelled(self):
        with self.lock:
            self.cancel_gesture(self.larm_gh)
            self.larm_gh = None

    def rarm_cancelled(self):
        with self.lock:
            self.cancel_gesture(self.rarm_gh)
            self.rarm_gh = None

    def cancel_gesture(self, goal_handle):
        with self.lock:
            gesture_handle = self.get_gesture_handle(goal_handle)
            gesture_handle.stop_timer()

            if self.is_keyframe_animation(gesture_handle.gesture):
                self.motion_proxy.stop(gesture_handle.motion_id)
            else:
                gesture_handle.client.cancel_goal()

            self.remove_gesture_handle(gesture_handle)

if __name__ == "__main__":
    rospy.init_node('gesture_action_server')
    gesture_server = NaoGestureActionServer()
    gesture_server.start()
    rospy.spin()
