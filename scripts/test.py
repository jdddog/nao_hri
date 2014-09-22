from enum import Enum



class IGesture(Enum):

    def __init__(self, default_duration):
        self.default_duration = default_duration

    def __new__(cls, *args):
        value = len(cls.__members__) + 1
        obj = object.__new__(cls)
        obj._value_ = value
        return obj


class Gesture(IGesture):

    LarmDown = (2.0, None)
    RarmDown = (2.0, None)
    WaveLarm = (6.0, None)
    MotionRight = (2.0, None)
    MotionLeft = (2.0, None)

    def __init__(self, default_duration, animation_type):
        IGesture.__init__(self, default_duration)
        self.animation_type = animation_type


print(Gesture.LarmDown.default_duration)
print(Gesture.LarmDown.animation_type)
print(Gesture.LarmDown.value)

