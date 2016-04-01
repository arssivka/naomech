from threading import Thread, Lock

import time
from enum import Enum


class Stance(Enum):
    FACE_DOWN = 0
    FACE_UP = 1
    RIGHT_SHOULDER_UP = 2
    LEFT_SHOULDER_UP = 3
    STAND = 4


class StanceDeterminator:
    _switcher = (
        lambda v: Stance.FACE_DOWN if v > 0.0 else Stance.FACE_UP,
        lambda v: Stance.LEFT_SHOULDER_UP if v > 0.0 else Stance.RIGHT_SHOULDER_UP,
        lambda v: Stance.STAND,
    )

    def __init__(self, robot):
        self.robot = robot

    def determinate(self):
        data = self.robot.accelerometer.acceleration()['data']
        index, val = max(enumerate(data), key=lambda x: abs(x[1]))
        return self._switcher[index](val)

class Behavior:
    def run(self):
        pass

    def reset(self):
        pass

    def is_done(self):
        return True

    def stop(self):
        pass


class UnknownBehavior(Behavior):
    def run(self):
        pass
        print "Warning: Unknown behavior!"


class StandingUpBehavior(Behavior):
    def __init__(self, robot, pose_handler, pose_switcher, stance_determinator):
        self.robot = robot
        self.pose_handler = pose_handler
        self.pose_switcher = pose_switcher
        self.stance_determinator = stance_determinator
        self._finished = False

    def reset(self):
        self.pose_switcher.stop()
        self._finished = False

    def run(self):
        if not self._finished:
            self.robot.joints.hardness(0.0)
            state = self.stance_determinator.determinate()
            time.sleep(1.0)
            while all(state != st for st in (Stance.FACE_DOWN, Stance.FACE_UP)):
                time.sleep(1.0)
            self.robot.joints.hardness(1.0)
            if state == Stance.FACE_DOWN:
                self.pose_switcher.switch_to("face_floor", "walking_pose")
            elif state == Stance.FACE_UP:
                self.pose_switcher.switch_to("face_up_init", "walking_pose")
            self.robot.joints.hardness(0.8)
            self._finished = True

    def is_done(self):
        return self._finished

    def stop(self):
        self.reset()




class BehaviorHandler:
    def __init__(self, robot, walker, pose_handler, pose_switcher):
        self.fall_indicator_count = 20
        self._robot = robot
        self._walker = walker
        self._pose_handler = pose_handler
        self._pose_switcher = pose_switcher
        self._stance_determinator = StanceDeterminator(robot)
        self._iterrupt = False
        self._behavior = UnknownBehavior()
        self._lock = Lock()
        self._worker = Thread(target=BehaviorHandler.__worker, args=(self,))
        self._worker.start()

    def __worker(self):
        self._lock.acquire()
        try:
            while not self._iterrupt:
                behavior = self._behavior
                self._lock.release()
                try:
                    behavior.run()
                finally:
                    self._lock.acquire()
        finally:
            self._lock.release()

    def __set_behavior(self, behavior, *args, **kwargs):
        self._lock.acquire()
        try:
            if not isinstance(self._behavior, behavior):
                self._behavior = behavior(*args, **kwargs)
        finally:
            self._lock.release()

    def run(self):
        counter = 0
        self.__set_behavior(UnknownBehavior)
        while not self._iterrupt:
            stance = self._stance_determinator.determinate()
            counter = counter + 1 if stance != Stance.STAND else 0
            # print type(self._behavior), stance, counter
            if counter >= self.fall_indicator_count:
                self.__set_behavior(StandingUpBehavior, self._robot, self._pose_handler,
                                    self._pose_switcher, self._stance_determinator)
                continue
            if any(isinstance(self._behavior, behavior) for behavior in (StandingUpBehavior,)) and \
                    not self._behavior.is_done():
                continue
            self.__set_behavior(UnknownBehavior)

    def stop(self):
        self._iterrupt = True
        self._worker.join()
