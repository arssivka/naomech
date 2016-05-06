from threading import Thread, Lock

import time

import math
from enum import Enum

from camera_geometry import CamGeom
from threadsafe import ThreadSafe
from walker import Walker


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
        print "Warning: Unknown behavior!"


class SwitcherBasedBehavior(Behavior):
    def __init__(self, robot, pose_handler, pose_switcher, stance_determinator, walker):
        self.robot = robot
        self.pose_handler = pose_handler
        self.pose_switcher = pose_switcher
        self.stance_determinator = stance_determinator
        self.walker = walker
        self._finished = False

    def prepare(self):
        pass

    def get_instances(self):
        raise NotImplementedError

    def finalize(self):
        pass

    def reset(self):
        self.pose_switcher.stop()
        self._finished = False

    def run(self):
        if not self._finished:
            self.walker.reset()
            self.prepare()
            switch = self.get_instances()
            if switch:
                aa = self.robot.locomotion.autoapply.enable()
                self.robot.locomotion.autoapply.enable(False)
                self.pose_switcher.switch_to(*switch)
                self.robot.locomotion.autoapply.enable(aa)
            self.finalize()
            self._finished = True

    def is_done(self):
        return self._finished

    def stop(self):
        self.pose_switcher.stop()
        self._finished = True


class StandingUpBehavior(SwitcherBasedBehavior):
    def prepare(self):
        self.robot.joints.hardness(0.0)
        self.state = self.stance_determinator.determinate()
        time.sleep(1.0)
        counter = 10
        while all(self.state != st for st in (Stance.FACE_DOWN, Stance.FACE_UP)) and counter > 0:
            time.sleep(1.0)
            self.state = self.stance_determinator.determinate()
            counter -= 1
        if counter <= 0:
            self.stop()
        self.robot.joints.hardness(0.85)

    def get_instances(self):
        if self.state == Stance.FACE_DOWN:
            return "face_floor", "walking_pose"
        elif self.state == Stance.FACE_UP:
            return "face_up_init", "walking_pose"

    def finalize(self):
        self.robot.joints.hardness(0.8)
        self.stop()


class KickBehavior(SwitcherBasedBehavior):
    _left_leg = True

    def prepare(self):
        self.robot.joints.hardness(0.8)

    def set_left_leg(self, left=True):
        self._left_leg = left

    def get_instances(self):
        return "prepare_left_kick" if self._left_leg else "prepare_right_kick", "walking_pose"

    def finalize(self):
        time.sleep(1.0)


class WalkBehavior(Behavior):
    STOP = 0
    SMART_GO_TO = 1
    LINEAR_GO_TO = 2
    GO_AROUND = 3

    def __init__(self, walker):
        self._state = WalkBehavior.STOP
        self._applied = False
        self._walker = walker
        self._args = None
        self._lock = Lock()

    def run(self):
        if not self._applied:
            with self._lock:
                (
                    lambda: self._walker.stop(),
                    lambda: self._walker.smart_go_to(*self._args),
                    lambda: self._walker.linear_go_to(*self._args),
                    lambda: self._walker.go_around(*self._args)
                )[self._state]()
                self._applied = True

    def smart_go_to(self, x, y, speed):
        self._upd_args(WalkBehavior.SMART_GO_TO, x, y, speed)

    def go_around(self, angle):
        self._upd_args(WalkBehavior.GO_AROUND, angle)

    def linear_go_to(self, x, y, theta):
        self._upd_args(WalkBehavior.LINEAR_GO_TO, x, y, theta)

    def _upd_args(self, state, *args):
        with self._lock:
            if self._walker.is_done() or self._state != state or self._args != args:
                self._applied = False
            self._state = state
            self._args = args

    def stop(self):
        self._state = WalkBehavior.STOP
        self._applied = True
        self._walker.stop()


class BehaviorHandler:
    sleep_time = 0.01

    def __init__(self, robot, walker, pose_handler, pose_switcher, cam, localization):
        self.fall_indicator_count = 5
        self._robot = robot
        self._walker = walker
        self._cam = cam
        self._localization = localization
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
                    time.sleep(self.sleep_time)
                finally:
                    self._lock.acquire()
        finally:
            self._lock.release()

    def __set_behavior(self, behavior, *args, **kwargs):
        with self._lock:
            if not isinstance(self._behavior, behavior):
                self._behavior.stop()
                self._behavior = behavior(*args, **kwargs)

    def run(self):
        counter = 0
        start = time.time()
        left_leg = True
        self.__set_behavior(UnknownBehavior)
        stance = self._stance_determinator.determinate()
        if stance == Stance.STAND:
            self._pose_handler.set_pose("walking_pose", 2.0)
        timer = 0
        ball_found = False
        ball = {
            "x": 0,
            "y": 0,
            "width": 0,
            "height": 0,
        }
        while not self._iterrupt:
            stance = self._stance_determinator.determinate()
            counter = counter + 1 if stance != Stance.STAND else 0
            if counter >= self.fall_indicator_count:
                self.__set_behavior(StandingUpBehavior, self._robot, self._pose_handler,
                                    self._pose_switcher, self._stance_determinator, self._walker)
                if self._behavior.is_done():
                    self._behavior.reset()
                time.sleep(self.sleep_time)
                continue
            if any(isinstance(self._behavior, behavior) for behavior in (StandingUpBehavior, KickBehavior)):
                if not self._behavior.is_done():
                    time.sleep(self.sleep_time)
                    continue
                else:
                    if isinstance(self._behavior, StandingUpBehavior):
                        self._localization.localization(True)
                    self.__set_behavior(UnknownBehavior)

            if timer % 2 == 0:
                self._robot.vision.updateFrame()
                ball = self._robot.vision.ballDetect()
                ball_found = ball["width"] == 0.0
                pix = self._cam.imagePixelToWorld(ball["x"] + ball["width"]/2, ball["y"], False)
            if not ball_found:
                self.__set_behavior(WalkBehavior, self._walker)
                self._behavior.go_around(math.pi)
            elif math.hypot(pix[0], pix[1]) > 250.0:
                self.__set_behavior(WalkBehavior, self._walker)
                self._walker.look_at(pix[0], pix[1])
                self._behavior.smart_go_to(pix[0], pix[1], 100)
            timer = timer + 1
            time.sleep(self.sleep_time)
        # elif t == 20.0:
            #     self.__set_behavior(KickBehavior, self._robot, self._pose_handler,
            #                         self._pose_switcher, self._stance_determinator, self._walker)
            #     self._behavior.set_left_leg(left_leg)
            #     left_leg = not left_leg

    def stop(self):
        self._iterrupt = True
        self._worker.join()
