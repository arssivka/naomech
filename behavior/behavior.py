from threading import Thread

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
    def tick_state(self):
        print "Warning: Unknown behavior!"

    def tick_leds(self):
        pass

    def reset(self):
        pass

    def is_done(self):
        return True

    def wait(self):
        pass


class StandingUpBehavior(UnknownBehavior):
    def __init__(self, robot, pose_handler, pose_switcher, stance_determinator):
        self.robot = robot
        self.pose_handler = pose_handler
        self.pose_switcher = pose_switcher
        self.stance_determinator = stance_determinator
        self.worker = None
        self.reset()

    def reset(self):
        if self.worker:
            self.worker.join()
        self.worker = Thread(target=StandingUpBehavior._worker, args=(self,))
        self.worker.start()

    def _worker(self):
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
        return self.pose_switcher.complete >= 1.0


class BehaviorHandler:
    class State(Enum):
        UNKNOWN = 0,
        STANDING_UP = 1

    def __init__(self, robot, walker, pose_handler, pose_switcher):
        self.robot = robot
        self.walker = walker
        self.pose_handler = pose_handler
        self.pose_switcher = pose_switcher
        self.stance_determinator = StanceDeterminator(robot)
        self.worker = None
        self.iterrupt = False
        self.fall_indicator_count = 10

    def _worker(self):
        State = BehaviorHandler.State
        counter = 0
        state = State.UNKNOWN
        behavior = UnknownBehavior()
        while not self.iterrupt:
            stance = self.stance_determinator.determinate()
            counter = counter + 1 if stance != Stance.STAND else 0
            if counter >= self.fall_indicator_count:
                state = State.STANDING_UP
            # Try to stand up
            if state == State.STANDING_UP:
                if not isinstance(behavior, StandingUpBehavior):
                    behavior.wait()
                    behavior = StandingUpBehavior(self.robot, self.pose_handler,
                                                  self.pose_switcher, self.stance_determinator)
                elif behavior.is_done():
                    state = State.UNKNOWN
                    behavior = UnknownBehavior()
                    continue
            behavior.tick_state()
            behavior.tick_leds()
        if behavior:
            behavior.wait()

    def run(self):
        if self.worker is None or not self.worker.isAlive():
            self.iterrupt = False
            self.worker = Thread(target=BehaviorHandler._worker, args=(self,))
            self.worker.start()

    def wait(self):
        if self.worker:
            self.worker.join()

    def stop(self):
        self.iterrupt = True
