#! /usr/bin/env python
import rospy
import tf
from collections import deque
import re
import math


class RingBuffer(deque):
    # Ring buffer implementation using deque, written by vegaseat
    # Sourced from: http://www.daniweb.com/forums/post202523-3.html

    """
    inherits deque, pops the oldest data to make room
    for the newest data when size is reached
    """
    def __init__(self, size):
        deque.__init__(self)
        self.size = size

    def full_append(self, item):
        deque.append(self, item)
        # full, pop the oldest item, left most item
        self.popleft()

    def append(self, item):
        deque.append(self, item)
        # max size reached, append becomes full_append
        if len(self) == self.size:
            self.append = self.full_append

    def get(self):
        """returns a list of size items (newest items)"""
        return list(self)


class Smoother():
    def __init__(self):
        node_name = rospy.get_name()
        #print "node_name: " + str(node_name)
        rospy.init_node("button_1_smoother")
        node_name = rospy.get_name()
        #print "node_name: " + str(node_name)

        rate = rospy.get_param("~rate", 14.429)
        self.history_size = rospy.get_param("~history_size", 5)
        self.thresh = rospy.get_param("~thresh", 0.05)

        try:
            self.input_topic = rospy.get_param("~input_topic")
        except:
            err = 'Please specify an input topic'
            rospy.logerr('Please specify an input topic')
            raise Exception(err)

        try:
            self.output_topic = rospy.get_param("~output_topic")
        except:
            err = 'Please specify an output topic'
            rospy.logerr(err)
            raise Exception(err)

        self.tf = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(rate)
        self.history = RingBuffer(self.history_size)

        self.parent = self.get_parent(self.input_topic)
        #print "PARENT: " + str(self.parent)


    def run(self):
        self.buffer()
        self.median_rot = self.get_median_rot()

        while not rospy.is_shutdown():
            (trans, rot) = self.get_transform(self.parent, self.input_topic)
            # print "xr: " + str(rot[0]) + "  yr: " + str(rot[1]) + "  zr: " + str(rot[2]) + "  ?:" + str(rot[3])
            self.history.append((trans, rot))

            (m_trans, m_rot) = self.mean()
            dev_rot = self.dev_from_cur_rot(self.median_rot)

            # print "Rot: " + str(self.median_rot)
            # print "Dev: " + str(dev_rot)

            if dev_rot[0] > self.thresh or dev_rot[1] > self.thresh or dev_rot[2] > self.thresh or dev_rot[3] > self.thresh:
                self.median_rot = self.get_median_rot()

            self.br.sendTransform(m_trans, self.median_rot, rospy.Time.now(), self.output_topic, self.parent)
            # self.br.sendTransform((10.0, 0.0, 0.0), (0.0,0.0,0.0,1.0), rospy.Time.now(), "carrot" , self.new_name)
            self.rate.sleep()

    def get_frames(self):
        while True:
            self.rate.sleep()
            frames = self.tf.allFramesAsString()

            if frames != "":
                break

        return frames

    def dev_from_cur_rot(self, cur_rot):
        rot = [0.0, 0.0, 0.0, 0.0]

        for (t1, r1) in self.history:
            rot[0] += math.pow(r1[0] - cur_rot[0], 2.0)
            rot[1] += math.pow(r1[1] - cur_rot[1], 2.0)
            rot[2] += math.pow(r1[2] - cur_rot[2], 2.0)
            rot[3] += math.pow(r1[3] - cur_rot[3], 2.0)

        length = len(self.history)
        rot[0] /= (length - 1)
        rot[1] /= (length - 1)
        rot[2] /= (length - 1)
        rot[3] /= (length - 1)

        rot[0] = math.sqrt(rot[0])
        rot[1] = math.sqrt(rot[1])
        rot[2] = math.sqrt(rot[2])
        rot[3] = math.sqrt(rot[3])

        return (rot[0], rot[1], rot[2], rot[3])

    def buffer(self):
        i = 0
        while not rospy.is_shutdown():
            if i < self.history_size:
                (trans, rot) = self.get_transform(self.parent, self.input_topic)
                self.history.append((trans, rot))
                i += 1
            else:
                break

            self.rate.sleep()

    def get_median_rot(self):
        x = []
        y = []
        z = []
        w = []
        middle = int(len(self.history) / 2)
        #print "middle: " + str(middle)
        for (trans, rot) in self.history:
            x.append(rot[0])
            y.append(rot[1])
            z.append(rot[2])
            w.append(rot[3])

        x.sort()
        y.sort()
        z.sort()
        w.sort()

        return (x[middle], y[middle], z[middle], w[middle])

    def mean(self):
        av_trans = [0.0, 0.0, 0.0]
        av_rot = [0.0, 0.0, 0.0, 0.0]

        num_points = float(len(self.history))
        # w = 1.0

        for (trans, rot) in self.history:
            av_trans[0] += trans[0]
            av_trans[1] += trans[1]
            av_trans[2] += trans[2]
            av_rot[0] += rot[0]
            av_rot[1] += rot[1]
            av_rot[2] += rot[2]
            av_rot[3] += rot[3]

        av_trans[0] /= num_points
        av_trans[1] /= num_points
        av_trans[2] /= num_points
        av_rot[0] /= num_points
        av_rot[1] /= num_points
        av_rot[2] /= num_points
        av_rot[3] /= num_points

        # br.sendTransform((0.0, 0.0, 0.127), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "gaze_base", "torso")

        return ((av_trans[0], av_trans[1], av_trans[2]), (av_rot[0], av_rot[1], av_rot[2], av_rot[3]))

    def get_parent(self, topic):
        prefix = "Frame " + topic.replace("/", "") + " exists with parent "
        #print "Prefix: " + prefix

        while not rospy.is_shutdown():
            frames = self.get_frames()
            #print "frames: " + frames
            hierarchy_info = re.search(prefix + "[A-Za-z_]+", frames)

            if hierarchy_info is not None:
                regex_result = hierarchy_info.group(0)
                break
            else:
                print "Looking for parent of topic:'" + topic + "'"

        parent = regex_result.replace(prefix, "")
        print "Parent of " + topic + " is: '" + str(parent) + "'"

        return parent

    def get_transform(self, origin, target):
        try:
            try:
                self.tf.waitForTransform(origin, target, rospy.Time(), rospy.Duration(5.0))
                (trans, rot) = self.tf.lookupTransform(origin, target, rospy.Time())
                return (trans, rot)
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("tf Failure")
        except tf.Exception:
            print "Couldn't transform"

if __name__ == '__main__':
    smoother = Smoother()
    smoother.run()
    rospy.spin()
