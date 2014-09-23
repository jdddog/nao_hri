#!/usr/bin/env python
# Copyright (c) 2014, OpenCog Foundation
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
# * Neither the name of the OpenCog Foundation nor the names of its
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

__author__ = 'Jamie Diprose'


import rospy
from hri_framework import ITargetActionServer
from geometry_msgs.msg import PointStamped, Point
from std_srvs.srv import Empty
import tf
from math import sqrt
from ros_blender_bridge.srv import SetSpeed, SetAcceleration
from hri_framework.entity_proxy import EntityProxy
from std_msgs.msg import Bool, Empty as EmptyMsg



def interpolate(value, old_min, old_max, new_min, new_max):
    # Width of each range
    old_range = old_max - old_min
    new_range = new_max - new_min

    # Scale old value into range between 0 and 1
    scaled_value = (value - old_min) / old_range

    # Convert the scaled value into the new range
    new_val = new_min + (scaled_value * new_range)

    return new_val


class BlenderTargetServer(ITargetActionServer):

    MIN_ITERATIONS_STOPPED = 5

    def __init__(self):
        self.action_server_name = rospy.get_param('~action_server_name')
        super(BlenderTargetServer, self).__init__(self.action_server_name)

        self.tl = tf.TransformListener()
        self.controller_name = rospy.get_param('~controller_name')
        self.target_name = rospy.get_param('~target_name')
        self.axes = rospy.get_param('~axes')
        self.joints_stopped = False

        self.gaze_target_pub = rospy.Publisher(self.controller_name + '/' + self.target_name, PointStamped, queue_size=1)
        self.enable_srv = rospy.ServiceProxy(self.controller_name + '/enable', Empty)
        self.disable_srv = rospy.ServiceProxy(self.controller_name + '/disable', Empty)
        self.speed_srv = rospy.ServiceProxy(self.controller_name + '/set_speed', SetSpeed)
        self.accel_srv = rospy.ServiceProxy(self.controller_name + '/set_acceleration', SetAcceleration)

        rospy.Subscriber(self.controller_name + '/joints_stopped', EmptyMsg, self.joints_stopped_callback)

    def joints_stopped_callback(self, msg):
        self.joints_stopped = True

    def get_axis_index(self, name):
        if name == 'x':
            return 0
        elif name == 'y':
            return 1
        elif name == 'z':
            return 2
        else:
            raise Exception('not a valid axis name: {0}'.format(name))

    def execute(self, gaze_goal):
        self.enable_srv()
        self.speed_srv(gaze_goal.speed)
        self.speed_srv(interpolate(gaze_goal.speed, 0.0, 1.0, 0.0, 0.3))
        self.joints_stopped = False
        count = 0

        while not rospy.is_shutdown() and not self.action_server.is_preempt_requested() and self.action_server.is_active():
            entity = EntityProxy(gaze_goal.target)
            entity_tf_frame = entity.tf_frame()

            try:
                (target_trans, target_rot) = self.tl.lookupTransform(self.origin_frame, entity_tf_frame, rospy.Time(0))
                point_stamped = PointStamped()
                point_stamped.header.frame_id = self.origin_frame
                point_stamped.header.stamp = rospy.Time().now()
                point_stamped.point = Point(x=target_trans[0], y=target_trans[1], z=target_trans[2])
                self.gaze_target_pub.publish(point_stamped)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            try:
                if self.joints_stopped:
                    count += 1

                (curr_trans, curr_rot) = self.tl.lookupTransform(self.end_effector_frame, entity_tf_frame, rospy.Time(0))

                i = self.get_axis_index(self.axes[0])
                j = self.get_axis_index(self.axes[1])

                y = curr_trans[i]
                z = curr_trans[j]
                distance_to_target = sqrt(y*y + z*z)
                rospy.loginfo('gaze_frame: {0}, entity_tf_frame: {1}, y: {2}, z: {3}, distance: {4}'.format(self.end_effector_frame, entity_tf_frame, y, z, distance_to_target))
                self.send_feedback(distance_to_target)

                print("STOPPED: " + str(self.joints_stopped))

                if distance_to_target < self.success_distance or (self.joints_stopped and count < BlenderTargetServer.MIN_ITERATIONS_STOPPED):
                    self.action_server.set_succeeded()
                    print("SUCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCEEEEEEEEEEEEEEEEEEEEEEEDDDDDDDDDDDDDEEEEEEEEEEEEEEEEEEDDDDDDDDDDDDDDDDDDDDd")
                    break

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            self.rate.sleep()

        self.disable_srv()


                    #or self.target_reached:
                    # #TODO: check min max motor positions because person could be out of bounds of gaze

if __name__ == '__main__':
    rospy.init_node('blender_target_server')
    server = BlenderTargetServer()
    server.start()
    rospy.spin()