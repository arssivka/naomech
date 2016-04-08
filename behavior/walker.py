import math
import time
from operator import itemgetter
from threading import Thread

from enum import Enum


class WalkState(Enum):
    SO_MUCH_ROTATE = 0
    MUCH_ROTATE = 1
    ROTATE = 2
    GO_CURVE = 3
    GO_STRAIGHT = 4


class Walker:
    keys = ['X', 'Y', 'THETA']
    sleep_time = 0.2

    def __init__(self, robot):
        self.robot = robot
        self.target = (0.0, 0.0)
        self.speed = 0.0
        self.worker = Thread()
        self.odo = [0.0, 0.0, 0.0]
        self.position_inaccuracy = 70.0
        self.angle_inaccuracy = math.radians(25.0)
        self.max_angular_speed = math.radians(10.0)

    def _worker(self):
        target = (0.0, 0.0)
        speed = 0.0
        q = None
        state = WalkState.SO_MUCH_ROTATE
        self.robot.locomotion.odometry(True)
        autoapply_enabled = self.robot.locomotion.autoapply.enable()
        while self.speed != 0.0 and self.target != (0.0, 0.0):
            if speed != self.speed:
                speed = self.speed
                r = speed / self.max_angular_speed
                update_q = True
            if target != self.target:
                target = self.target
                angle = math.atan2(target[1], target[0])
                self.odo = [0.0, 0.0, 0.0]
                update_q = True
                self.robot.locomotion.odometry(True)
            else:
                self.odo = self.robot.locomotion.odometry()['data']
            a = angle - self.odo[2]
            # a = min(a, 2 * math.pi - a, key=abs)
            dist_left = (target[0] - self.odo[0],
                         target[1] - self.odo[1])
            if update_q and abs(a) < self.angle_inaccuracy:
                q_origin = (self.odo[0], self.odo[1])
                q_target = (target[0], target[1])
                sign = (1.0, -1.0)
                angle_to_p = tuple(self.odo[2] - math.pi / 2.0 * s for s in sign)
                p = tuple((q_origin[0] + r * math.cos(angle),
                           (q_origin[1] + r * math.sin(angle))) for angle in angle_to_p)
                d = tuple((q_target[0] - point[0], q_target[1] - point[1]) for point in p)
                h = tuple(math.hypot(d_i[0], d_i[1]) for d_i in d)
                index, _ = min(enumerate(h), key=itemgetter(1))
                if r > h[index]:
                    q = None
                else:
                    phi = math.atan2(d[index][1], d[index][0])
                    q_theta = math.acos(r / h[index])
                    q = (p[index][0] + r * math.cos(phi + q_theta * sign[index]),
                         (p[index][1] + r * math.sin(phi + q_theta * sign[index])))
                    theta = -sign[index] * self.max_angular_speed
                    update_q = False
            if any(state == s for s in (WalkState.SO_MUCH_ROTATE, WalkState.MUCH_ROTATE)):
                pi3 = math.pi / 3.0
                abbabbbebe = math.copysign(1.0, angle) # It's rotate direction. We haven't any ideas about its name :D
                if abs(self.odo[2]) >= pi3:
                    if state == WalkState.MUCH_ROTATE:
                        state = WalkState.ROTATE
                    else:
                        state = WalkState.MUCH_ROTATE
                    c = math.cos(-pi3 * abbabbbebe)
                    s = math.sin(-pi3 * abbabbbebe)
                    self.target = (target[0] * c - target[1] * s,
                                   target[0] * s + target[1] * c)
                    continue
                if angle < pi3:
                    state = WalkState.ROTATE
                    continue
                elif angle < pi3 * 2.0:
                    state = WalkState.MUCH_ROTATE
                    continue
                self.robot.locomotion.parameters(self.keys, [0.0, 0.0, self.max_angular_speed * abbabbbebe])
            elif state == WalkState.ROTATE:
                update_q = True
                if q and abs(a) < self.angle_inaccuracy:
                    state = WalkState.GO_CURVE
                    continue
                self.robot.locomotion.parameters(self.keys, [0.0, 0.0, math.copysign(self.max_angular_speed, a)])
            elif state == WalkState.GO_CURVE:
                if math.hypot(dist_left[0], dist_left[1]) < self.position_inaccuracy:
                    self.robot.locomotion.parameters(self.keys, [0.0, 0.0, 0.0])
                    break
                dist_left = (q[0] - self.odo[0],
                             q[1] - self.odo[1])
                if math.hypot(dist_left[0], dist_left[1]) < self.position_inaccuracy:
                    state = WalkState.GO_STRAIGHT
                    continue
                self.robot.locomotion.parameters(self.keys, [self.speed, 0.0, theta])
            else:
                if math.hypot(dist_left[0], dist_left[1]) < self.position_inaccuracy or math.hypot(self.odo[0], self.odo[1]) >= math.hypot(target[0], target[1]):
                    self.robot.locomotion.parameters(self.keys, [0.0, 0.0, 0.0])
                    break
                self.robot.locomotion.parameters(self.keys, [self.speed, 0.0, 0.0])
            self.robot.locomotion.autoapply.enable(True)
            time.sleep(self.sleep_time)
        while not self.robot.locomotion.is_done():
            time.sleep(0.1)
        self.robot.locomotion.autoapply.enable(autoapply_enabled)

    def go_around(self, point, speed):
        r = math.hypot(point[0], point[1])
        c = point[0] / r
        s = point[1] / r
        self.set_speed(speed * s, speed * c, -speed / r)

    def set_speed(self, x, y, theta):
        if self.worker.isAlive():
            self.speed = 0.0
            self.worker.join()
        values = [float(x), float(y), float(theta)]
        self.robot.locomotion.parameters(self.keys, values)
        self.robot.locomotion.autoapply.enable(True)

    def get_speed(self):
        return tuple(self.robot.locomotion.parameters(self.keys)['data'])

    def reset(self):
        self.speed = 0.0
        self.target = (0.0, 0.0)
        self.robot.locomotion.reset(True)

    def stop(self):
        self.speed = 0.0
        self.target = (0.0, 0.0)
        self.set_speed(0.0, 0.0, 0.0)
        while not self.robot.locomotion.is_done():
            if any(v != 0.0 for v in self.get_speed()):
                self.set_speed(0.0, 0.0, 0.0)
            time.sleep(self.sleep_time)

    def go_to(self, target, speed):
        if self.speed == 0.0 and self.worker.isAlive():
            self.worker.join()
        self.target = target
        self.speed = speed
        self.worker = Thread(target=Walker._worker, args=(self,))
        self.worker.start()