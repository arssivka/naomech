from itertools import izip
from threading import Thread, Lock

import time

import math
from enum import Enum

from localization import RobotPose
from camera_geometry import CamGeom
from threadsafe import ThreadSafe
from walker import Walker, OdoListener


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
        # print "Warning: Unknown behavior!"


class SwitcherBasedBehavior(Behavior):
    def __init__(self, robot, pose_handler, pose_switcher, stance_determinator, walker):
        self.robot = robot
        self.pose_handler = pose_handler
        self.pose_switcher = pose_switcher
        self.stance_determinator = stance_determinator
        self.walker = walker
        self._finished = False
        self._play = False

    def prepare(self):
        pass

    def get_instances(self):
        raise NotImplementedError

    def finalize(self):
        pass

    def reset(self):
        self.pose_switcher.stop()
        self._finished = False
        self._play = False

    def run(self):
        if not self._finished and self._play:
            self.walker.stop()
            self.prepare()
            self.robot.joints.hardness(0.9)
            switch = self.get_instances()
            if switch:
                aa = self.robot.locomotion.autoapply.enable()
                self.robot.locomotion.autoapply.enable(False)
                self.pose_switcher.switch_to(*switch)
                self.robot.locomotion.autoapply.enable(aa)
            self.finalize()
            self._finished = True

    def start(self):
        self._play = True

    def is_done(self):
        return self._finished

    def stop(self):
        self.pose_switcher.stop()
        self._finished = True
        self._play = False


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

    def go_around(self, angle, scale=1.0):
        self._upd_args(WalkBehavior.GO_AROUND, angle, scale)

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

    def is_done(self):
        return self._walker.is_done()


class BehaviorHandler(OdoListener):
    FINDING_SECTOR_ANGLE = math.radians(60.0)
    HEAD_PITCH_STEP = math.radians(30.0)
    HEAD_YAW_STEP = math.radians(15.0)
    SLEEP_TIME = 0.01

    def __init__(self, robot, walker, pose_handler, pose_switcher, cam, localization):
        self.fall_indicator_count = 3
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
        self.odo = RobotPose(0.0, 0.0, 0.0)

    def notify(self, frodo):
        self.odo.odoTranslate(frodo[0], frodo[1], frodo[2])

    def __worker(self):
        self._lock.acquire()
        try:
            while not self._iterrupt:
                behavior = self._behavior
                self._lock.release()
                try:
                    behavior.run()
                    time.sleep(self.SLEEP_TIME)
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
        timestamp = 0
        pix = [0.0, 0.0]
        reached = False
        dode = 0
        fff = False
        initialized = False
        tuc = None
        print "Behavior was started"
        while not self._iterrupt:
            stance = self._stance_determinator.determinate()
            counter = counter + 1 if stance != Stance.STAND else 0
            if counter >= self.fall_indicator_count:
                dode = 0
                self.__set_behavior(StandingUpBehavior, self._robot, self._pose_handler,
                                    self._pose_switcher, self._stance_determinator, self._walker)
                if self._behavior.is_done():
                    self._behavior.reset()
                self._behavior.start()
                timer = timer + 1
                time.sleep(self.SLEEP_TIME)
                continue
            if any(isinstance(self._behavior, behavior) for behavior in (StandingUpBehavior, KickBehavior,)):
                if not self._behavior.is_done():
                    timer = timer + 1
                    time.sleep(self.SLEEP_TIME)
                    continue
                else:
                    if isinstance(self._behavior, StandingUpBehavior):
                        self._localization.localization(True)
                    self.__set_behavior(UnknownBehavior)
            if timestamp + 24 < timer or pix == [0.0, 0.0, 0.0]:
                reached = False

            self._robot.vision.updateFrame()
            ball = self._robot.vision.ballDetect()
            ball_found = (ball["width"] != 0.0)
            if ball_found:
                timestamp = timer
                pix = self._cam.imagePixelToWorld(ball["x"] + ball["width"]/2, ball["y"], False)
            # dode = False
            # if not initialized:
            #     print "go to start"
            #     if tuc is None or math.hypot(tuc[0], tuc[1]) > 100:
            #         tuc = self._localization.global_to_local(self._localization.map.start_point.x, self._localization.map.start_point.y)
            #         self.__set_behavior(WalkBehavior, self._walker)
            #         self._behavior.smart_go_to(tuc[0], tuc[1], 100)
            #         self._walker.look_at(500.0, 0.0, 0.0)
            #         time.sleep(2.0)
            #         timer += 1
            #         continue
            #     else:
            #         initialized = True

            if ball_found:
                print "FOUND!!!!!!"
            if reached or dode > 0:
                fff = False
                bdone = self._behavior.is_done()
                if ball_found:
                    if pix[0] > 0.0:
                        self._walker.look_at(pix[0], pix[1])
                if dode == 0:
                    enemy_point = self._localization.map.enemy_point
                    gates = self._localization.global_to_local(enemy_point.x, enemy_point.y)
                    self.__set_behavior(WalkBehavior, self._walker)
                    if gates[2] > math.radians(30):
                        self._behavior.go_around(gates[2])
                        dode = 1
                        bdone = False
                        time.sleep(0.2)
                    else:
                        dode = 1
                        bdone = True
                if dode == 1 and bdone:
                    self.__set_behavior(KickBehavior, self._robot, self._pose_handler,
                                        self._pose_switcher, self._stance_determinator, self._walker)
                    self._behavior.set_left_leg(pix[1] > 0)
                    aa = self._robot.locomotion.autoapply.enable(False)
                    self._behavior.start()
                    dode = 2
                    bdone = False
                    time.sleep(0.2)
                if dode == 2 and bdone:
                    dode = 0
                    time.sleep(0.5)

            if dode == 0:
                if timestamp == 0 or timestamp + 54 < timer or pix == [0.0, 0.0, 0.0] or pix[0] < 0.0:
                    self.__set_behavior(WalkBehavior, self._walker)
                    if not ball_found:
                        if fff == False:
                            h_angle = 0.0
                            low = True
                            to_left = True
                            fff = True
                        print "h_angle", math.degrees(h_angle), to_left, low,
                        if to_left:
                            if h_angle == -self.FINDING_SECTOR_ANGLE / 2.0:
                                low = True
                                to_left = False
                                h_angle += self.HEAD_YAW_STEP
                            else:
                                h_angle -= self.HEAD_YAW_STEP
                        else:
                            if h_angle == self.FINDING_SECTOR_ANGLE / 2.0:
                                low = False
                                to_left = True
                                h_angle -= self.HEAD_YAW_STEP
                            else:
                                h_angle += self.HEAD_YAW_STEP
                        h_y = 0.0
                        h_x = 300.0 if low else 1000.0
                        c = math.cos(h_angle)
                        s = math.sin(h_angle)
                        h_x, h_y = h_x * c - h_y * s, h_x * s + h_y * c
                        print "h x, y", h_x, h_y
                        self._walker.look_at(h_x, h_y, 0.0)
                    else:
                        print "!!!"
                        if pix[0] > 0.0:
                            self._walker.look_at(pix[0], pix[1])
                    self._behavior.go_around(math.pi, 0.5)
                elif math.hypot(pix[0], pix[1]) > 350.0:
                    fff = False
                    # print pix
                    if pix[0] > 0.0:
                        self._walker.look_at(pix[0], pix[1])
                    enemy_point = self._localization.map.enemy_point
                    gates = self._localization.global_to_local(enemy_point.x, enemy_point.y)
                    dx = pix[0] - gates[0]
                    dy = pix[1] - gates[1]
                    angle = math.atan2(dy, dx)
                    distance = math.hypot(dx, dy)
                    new_dist = distance + 180
                    target = (pix[0] + math.cos(angle) * (new_dist) - dx, pix[1] + math.sin(angle) * (new_dist) - dy)
                    print "target", target
                    self.__set_behavior(WalkBehavior, self._walker)
                    print repr(self._behavior)
                    self._behavior.smart_go_to(target[0], target[1], 100)
                elif pix != [0.0, 0.0, 0,0] and  pix[0] > 0.0:
                    fff = False
                    reached = True
            timer = timer + 1
            time.sleep(self.SLEEP_TIME)
            # elif t == 20.0:
            #     self.__set_behavior(KickBehavior, self._robot, self._pose_handler,
            #                         self._pose_switcher, self._stance_determinator, self._walker)
            #     self._behavior.set_left_leg(left_leg)
            #     left_leg = not left_leg

    def stop(self):
        self._iterrupt = True
        self._worker.join()

class GoalieBehaviourHandler(BehaviorHandler):
    def run(self):
        la_coords = [1000.0, 0.0]
        step = 100.0
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
        timestamp = 0
        pix = [0.0, 0.0]
        reached = False
        dode = 0
        initialized = False
        while not self._iterrupt:
            stance = self._stance_determinator.determinate()
            counter = counter + 1 if stance != Stance.STAND else 0
            if counter >= self.fall_indicator_count:
                dode = 0
                self.__set_behavior(StandingUpBehavior, self._robot, self._pose_handler,
                                    self._pose_switcher, self._stance_determinator, self._walker)
                if self._behavior.is_done():
                    self._behavior.reset()
                timer = timer + 1
                time.sleep(self.sleep_time)
                continue
            if any(isinstance(self._behavior, behavior) for behavior in (StandingUpBehavior, KickBehavior)):
                if not self._behavior.is_done():
                    timer = timer + 1
                    time.sleep(self.sleep_time)
                    continue
                else:
                    if isinstance(self._behavior, StandingUpBehavior):
                        self._localization.localization(True)
                    self.__set_behavior(UnknownBehavior)

            self._robot.vision.updateFrame()
            ball = self._robot.vision.ballDetect()
            ball_found = (ball["width"] != 0.0)
            if ball_found:
                timestamp = timer
                pix = self._cam.imagePixelToWorld(ball["x"] + ball["width"]/2, ball["y"], False)
            if timestamp + 8 < timer or pix == [0.0, 0.0, 0.0] or pix[0] < 0.0:
                print "finding"
                self._robot.locomotion.autoapply.enable(False)
                la_coords[1] += step
                self._robot.kinematics.lookAt(la_coords[0], la_coords[1], 0.0, False)
                if abs(la_coords[1]) >= 1000.0:
                    step *= -1
                    self._pose_handler.set_pose("walking_pose", 1.0)
                    la_coords[1] = 0
            elif abs(pix[1]) > 100.0:
                print pix
                self._robot.locomotion.autoapply.enable(True)
                self.__set_behavior(WalkBehavior, self._walker)
                self._walker.look_at(pix[0], pix[1])
                tmp_p = self._localization.position.point.y + pix[1]
                if tmp_p > 1100.0 or tmp_p < -1100.0:
                    self._behavior.linear_go_to(0.0, math.copysign((1100.0 - abs(tmp_p)), pix[1]), 100.0)
                else:
                    self._behavior.inear_go_to(0.0, pix[1], 100.0)
                time.sleep(0.3)
            elif math.hypot(pix[0], pix[1]) < 200.0:
                self.__set_behavior(KickBehavior, self._robot, self._pose_handler,
                                    self._pose_switcher, self._stance_determinator, self._walker)
                self._behavior.set_left_leg(pix[1] > 0)
                aa = self._robot.locomotion.autoapply.enable(False)
                self._behavior.start()
                time.sleep(0.3)
            timer = timer + 1
            time.sleep(self.sleep_time)

    def __set_behavior(self, behavior, *args, **kwargs):
        with self._lock:
            if not isinstance(self._behavior, behavior):
                self._behavior.stop()
                self._behavior = behavior(*args, **kwargs)

