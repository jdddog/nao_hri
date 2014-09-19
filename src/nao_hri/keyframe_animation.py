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

import math
from copy import deepcopy


def limit(value, minimum, maximum):
    if value < minimum:
        return minimum
    elif value > maximum:
        return maximum
    else:
        return value


def min_limit(value, minimum):
    if value < minimum:
        return minimum
    else:
        return value


def max_limit(value, maximum):
    if value > maximum:
        return maximum
    else:
        return value


def convert_range(value, oldStart, oldEnd, newStart, newEnd):
    scale = (newEnd - newStart) / (oldEnd - oldStart)
    return newStart + ((value - oldStart) * scale)


class KeyframeAnimation():

    def __init__(self, names, times, keys):
        self.names = names
        self.times = times
        self.keys = keys

    def get_start_time(self):
        minTime = float('Inf')

        for time in self.times:
            tempMin = min(time)

            if tempMin < minTime:
                minTime = tempMin

        return minTime

    def get_end_time(self):
        maxTime = 0.0

        for time in self.times:
            tempMax = max(time)

            if tempMax > maxTime:
                maxTime = tempMax

        return maxTime

    #Desired time in seconds
    def get_ntk(self, desired_time):
        raise NotImplementedError("Please Implement this method: returns names, times and keys for a gesture")


class AtomicAnimation(KeyframeAnimation):

    def __init__(self, names, times, keys, min_time, max_time):
        KeyframeAnimation.__init__(self, names, times, keys)
        self.min_time = min_time
        self.max_time = max_time

    '''
        Desired time in seconds
    '''
    def use_desired_time(self):
        single_frame = True
        same_values = True
        times = []

        for time in self.times:
            if len(time) > 1:
                single_frame = False
                break
            else:
                times.append(time[0])

        if single_frame:
            first = times[0]

            for time in times:
                if time != first:
                    same_values = False
                    break

        return single_frame and same_values

    def get_ntk(self, desired_time):
        desired_time = limit(desired_time, self.min_time, self.max_time)
        start_time = self.get_start_time()
        end_time = self.get_end_time()
        new_animation_times = []
        use_dt = self.use_desired_time()

        for oldTime in self.times:
            newTime = []
            for value in oldTime:
                if use_dt:
                    newTime.append(desired_time)
                else:
                    newTime.append(convert_range(value, 0.0, end_time, 0.0, desired_time))

            new_animation_times.append(newTime)

        return (self.names, new_animation_times, self.keys)


class RepeatingAnimation(KeyframeAnimation):

    def __init__(self, names, times, keys, min_repeats):
        KeyframeAnimation.__init__(self, names, times, keys)
        self.min_repeats = min_repeats

    '''
        Desired time in seconds
    '''

    def get_ntk(self, desired_time):
        endTime = self.get_end_time()
        desired_time = limit(desired_time, endTime, desired_time)
        numRepeats = min_limit(int(math.ceil(desired_time / endTime)) - self.min_repeats, 2)
        newTimes = deepcopy(self.times)
        newKeys = deepcopy(self.keys)

        for n in range(1, numRepeats):
            for i, oldTime in enumerate(self.times):
                newTime = newTimes[i]
                maxValue = max(newTime)

                for value in oldTime:
                    newTime.append(value + maxValue)

            for i, oldKey in enumerate(self.keys):
                newKey = newKeys[i]

                for value in oldKey:
                    newKey.append(value)


        return (self.names, newTimes, newKeys)


