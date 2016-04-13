import math
import time
from copy import copy
from operator import itemgetter
from threading import Thread, Lock

from enum import Enum


class WalkState(Enum):
    SO_MUCH_ROTATE = 0
    MUCH_ROTATE = 1
    ROTATE = 2
    GO_CURVE = 3
    GO_STRAIGHT = 4


class Walker:
    _keys = ['X', 'Y', 'THETA']
    sleep_time = 0.2
    max_angular_speed = math.radians(10.0)
    angle_inaccuracy = math.radians(25.0)
    position_inaccuracy = 70.0
    _pi3 = math.pi / 3.0

    def __init__(self, robot):
        self._robot = robot
        self._target = (0.0, 0.0)
        self._speed = 0.0
        self._worker = Thread()
        self._odo = [0.0, 0.0, 0.0]
        self._mutex = Lock()

    def _thread_safe_get(self, func):
        self._mutex.acquire()
        try:
            return func()
        finally:
            self._mutex.release()

    def _thread_safe_set(self, variable, value):
        self._mutex.acquire()
        try:
            self.__dict__[variable] = value
        finally:
            self._mutex.release()

    def _set_speed(self, speed):
        self._thread_safe_set('_speed', speed)

    def _set_target(self, target):
        self._thread_safe_set('_target', target)

    def _set_odo(self, odo):
        self._thread_safe_set('_odo', odo)

    def _get_speed(self):
        return self._thread_safe_get(lambda: self._speed)

    def _get_target(self):
        return self._thread_safe_get(lambda: copy(self._target))

    def _get_odo(self):
        return self._thread_safe_get(lambda: copy(self._odo))

    def _worker(self):
        target = (0.0, 0.0)
        speed = 0.0
        q = None
        state = WalkState.SO_MUCH_ROTATE
        autoapply_enabled = self._robot.locomotion.autoapply.enable()
        while self._get_speed() != 0.0 and self._get_target() != (0.0, 0.0):
            _speed = self._get_speed()
            _target = self._get_target()
            # Update speed
            if speed != _speed:
                speed = _speed
                r = speed / self.max_angular_speed
                update_q = True
            # Update target
            if target != _target:
                target = _target
                angle = math.atan2(target[1], target[0])
                self._set_odo([0.0, 0.0, 0.0])
                _odo = [0.0, 0.0, 0.0]
                update_q = True
                state = WalkState.SO_MUCH_ROTATE
                # Reset odometry
                self._robot.locomotion.odometry(True)
            else:
                _odo = self._robot.locomotion.odometry().get('data')
                self._set_odo(_odo)
            a = angle - _odo[2]
            dist_left = (target[0] - _odo[0],
                         target[1] - _odo[1])
            if update_q and abs(a) < self.angle_inaccuracy:
                # q_origin = (_odo[0], _odo[1])
                q_target = (target[0], target[1])
                sign = (1.0, -1.0)
                angle_to_p = tuple(_odo[2] - math.pi / 2.0 * s for s in sign)
                p = tuple((_odo[0] + r * math.cos(angle),
                           (_odo[1] + r * math.sin(angle))) for angle in angle_to_p)
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
            # Rotate at an angle greater than PI / 3
            # print 'state = ', str(state), ', q = ', q, ', odo = ', _odo, ', target = ', target
            if any(state == s for s in (WalkState.SO_MUCH_ROTATE, WalkState.MUCH_ROTATE)):
                abbabbbebe = math.copysign(1.0, angle) # It's rotate direction. We haven't any ideas about its name :D
                if abs(_odo[2]) >= self._pi3:
                    if state == WalkState.MUCH_ROTATE:
                        state = WalkState.ROTATE
                    else:
                        state = WalkState.MUCH_ROTATE
                    c = math.cos(-self._pi3 * abbabbbebe)
                    s = math.sin(-self._pi3 * abbabbbebe)
                    self._set_target((target[0] * c - target[1] * s,
                                      target[0] * s + target[1] * c))
                    continue
                if angle < self._pi3:
                    state = WalkState.ROTATE
                    continue
                elif angle < self._pi3 * 2.0 and state != WalkState.MUCH_ROTATE:
                    state = WalkState.MUCH_ROTATE
                    continue
                # Speed update
                self._robot.locomotion.parameters(self._keys, [0.0, 0.0, self.max_angular_speed * abbabbbebe])
            # Rotate to a target
            elif state == WalkState.ROTATE:
                if q and abs(a) < self.angle_inaccuracy:
                    state = WalkState.GO_CURVE
                    continue
                update_q = True
                # Speed update
                self._robot.locomotion.parameters(self._keys, [0.0, 0.0, math.copysign(self.max_angular_speed, a)])
            # Go to target along curve
            elif state == WalkState.GO_CURVE:
                # Target reached check
                if math.hypot(dist_left[0], dist_left[1]) < self.position_inaccuracy:
                    self._robot.locomotion.parameters(self._keys, [0.0, 0.0, 0.0])
                    break
                # Q reached check
                dist_left = (q[0] - _odo[0], q[1] - _odo[1])
                if math.hypot(dist_left[0], dist_left[1]) < self.position_inaccuracy:
                    state = WalkState.GO_STRAIGHT
                    continue
                # Speed update
                self._robot.locomotion.parameters(self._keys, [_speed, 0.0, theta])
            # Go straight
            else:
                # Target reached check
                if math.hypot(dist_left[0], dist_left[1]) < self.position_inaccuracy or \
                                math.hypot(_odo[0], _odo[1]) >= math.hypot(target[0], target[1]):
                    break
                # Speed update
                self._robot.locomotion.parameters(self._keys, [_speed, 0.0, 0.0])
            self._robot.locomotion.autoapply.enable(True)
            time.sleep(self.sleep_time)
        self._robot.locomotion.parameters(self._keys, [0.0, 0.0, 0.0])
        # Wait until gait is done
        self._set_speed(0.0)
        self._set_target((0.0, 0.0))
        while not self._robot.locomotion.is_done() \
                and self._robot.locomotion.parameters(self._keys).get('data') == [0.0, 0.0, 0.0]:
            time.sleep(self.sleep_time)
        if self._robot.locomotion.parameters(self._keys).get('data') == [0.0, 0.0, 0.0]:
            self._robot.locomotion.autoapply.enable(autoapply_enabled)

    def go_around(self, point, speed):
        r = math.hypot(point[0], point[1])
        c = point[0] / r
        s = point[1] / r
        self.set_speed(speed * s, speed * c, -speed / r)

    def set_speed(self, x, y, theta):
        if self._worker.isAlive():
            self._set_speed(0.0)
            self._worker.join()
        values = [float(x), float(y), float(theta)]
        self._robot.locomotion.parameters(self._keys, values)
        self._robot.locomotion.autoapply.enable(True)

    def get_speed(self):
        return self._robot.locomotion.parameters(self._keys).get('data')

    def reset(self):
        self._set_speed(0.0)
        self._set_target((0.0, 0.0))
        self._robot.locomotion.reset(True)

    def is_done(self):
        return not self._worker.isAlive()

    def stop(self):
        self._speed = 0.0
        self._target = (0.0, 0.0)
        self.set_speed(0.0, 0.0, 0.0)
        while not self._robot.locomotion.is_done():
            if any(v != 0.0 for v in self.get_speed()):
                self.set_speed(0.0, 0.0, 0.0)
            time.sleep(self.sleep_time)

    def go_to(self, target, speed):
        if self._speed == 0.0 and self._worker.isAlive():
            self._worker.join()
        self._set_target(target)
        self._set_speed(speed)
        if not self._worker.isAlive():
            self._worker = Thread(target=Walker._worker, args=(self,))
            self._worker.start()
