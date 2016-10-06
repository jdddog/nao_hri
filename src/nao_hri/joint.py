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
from ros_blender_bridge import Joint
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed


class NaoJoint(Joint):
    def __init__(self, joint_name):
        Joint.__init__(self, joint_name)
        self.msg = JointAnglesWithSpeed()
        self.msg.joint_names.append(joint_name)
        self.joint_pub = rospy.Publisher('joint_angles', JointAnglesWithSpeed, queue_size=10)

    def reset_msg(self):
        self.msg.joint_angles = []
        self.msg.header.stamp = rospy.Time().now()

    def set_position(self, position):
        if position not in self.msg.joint_angles:
            self.reset_msg()
            self.msg.joint_angles.append(position)
            self.msg.speed = self.speed
            self.joint_pub.publish(self.msg)

    def set_speed(self, speed):
        self.speed = speed

    def set_acceleration(self, acceleration):
        pass