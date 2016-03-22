from enum import Enum


class Stance(Enum):
    FACE_DOWN = 0,
    FACE_UP = 1,
    RIGHT_SHOULDER_UP = 2,
    LEFT_SHOULDER_UP = 3,
    STAND = 4


class StanceDeterminator:
    def __init__(self, robot):
        self.robot = robot

    def determinate(self):
        data = self.robot.accelerometer.acceleration()['data']
        index, v = max(enumerate(data), key=lambda x: abs(x[1]))
        return (
            lambda: Stance.FACE_DOWN if v > 0.0 else Stance.FACE_UP,
            lambda: Stance.LEFT_SHOULDER_UP if v > 0.0 else Stance.RIGHT_SHOULDER_UP,
            lambda: Stance.STAND
        )[index]()


class Fallmanager:
    count = 12

    def __init__(self, pose_handler, walker, stance_determinator):
        self.ph = pose_handler
        self.wlk = walker
        self.sd = stance_determinator

    def