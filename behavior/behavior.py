from threading import Thread, Lock

import time
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


class UnknownBehavior:
    def run(self):
        print "Warning: Unknown behavior!"

    def reset(self):
        pass

    def is_done(self):
        return True

    def stop(self):
        pass


class StandingUpBehavior(UnknownBehavior):
    def __init__(self, robot, pose_handler, pose_switcher, stance_determinator):
        self.robot = robot
        self.pose_handler = pose_handler
        self.pose_switcher = pose_switcher
        self.stance_determinator = stance_determinator

    def reset(self):
        self.pose_switcher.stop()

    def run(self):
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

    def is_done(self):
        return self.pose_switcher.is_done()

    def stop(self):
        self.reset()




class BehaviorHandler:
    def __init__(self, robot, walker, pose_handler, pose_switcher):
        self.fall_indicator_count = 10
        self._robot = robot
        self._walker = walker
        self._pose_handler = pose_handler
        self._pose_switcher = pose_switcher
        self._stance_determinator = StanceDeterminator(robot)
        self._iterrupt = False
        self._behavior = UnknownBehavior()
        self._lock = Lock()
        self._worker = Thread(target=BehaviorHandler.__worker, args=(self,))
        self._worker.run()

    def __worker(self):
        self._lock.acquire()
        try:
            while not self._iterrupt:
                behavior = self._behavior
                self._lock.release()
                behavior.run()
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
            if counter >= self.fall_indicator_count:
                self.__set_behavior(StandingUpBehavior, self._robot, self._walker, self._pose_handler, self._pose_switcher)
                continue
            self.__set_behavior(UnknownBehavior)

    def stop(self):
        self._iterrupt = True
        self._worker.join()
