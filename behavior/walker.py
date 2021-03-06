import math
import time
from abc import ABCMeta
from operator import itemgetter
from threading import Thread

from enum import Enum

from state_machine import ActionFactory, StateMachine
from state_machine import State
from threadsafe import ThreadSafe

LOCOMOTION_KEYS = ['X', 'Y', 'THETA']


class _WalkState(Enum):
    SO_MUCH_ROTATE = 0
    MUCH_ROTATE = 1
    ROTATE = 2
    GO_CURVE = 3
    GO_STRAIGHT = 4


class WalkCmd:
    """The structure contains constructors for actions

    This class is abstract and shouldn't created by call WalkCmd()
    """

    __metaclass__ = ABCMeta
    
    stop = ActionFactory("stop", args=("robot"))
    linear_go_to = ActionFactory("linear_set_target", args=("robot", "x", "y", "speed"))
    smart_go_to = ActionFactory("smart_set_target", args=("robot", "x", "y", "speed"))
    go_around = ActionFactory("set_rotation_target", args=("robot", "angle", "scale"))


class WalkingState(ThreadSafe, State):
    def __init__(self, ctx=None, action=None):
        ThreadSafe.__init__(self)
        State.__init__(self, ctx, action)
        self.update_robot(action)
        self.look_target = (0.0, 0.0, 0.0)
        self.set_thread_safe_all(("update_robot", "action", "odo_reset", "look_target"))

    def update_robot(self, action):
        robot = getattr(action, "robot", None)
        if robot is not None:
            self.robot = robot
        
    def set_autoupdate(self, state=True):
        self.robot.locomotion.autoapply.enable(state)
        
    def set_parameters(self, x, y, theta):
        global LOCOMOTION_KEYS
        self.robot.locomotion.parameters(LOCOMOTION_KEYS, [float(x), float(y), float(theta)])
        
    def get_parameters(self):
        return self.robot.locomotion.parameters(LOCOMOTION_KEYS).get("data")
        
    def get_odo(self):
        return self.robot.locomotion.odometry().get("data")

    def odo_reset(self):
        # Notify listeners about odometry will be changed.
        # Some systems may be sensitive to reset odometry
        odo = self.get_odo()
        self.robot.locomotion.odometry(True)
        listeners = self.ctx
        if listeners is not None:
            for lstr in listeners:
                lstr.notify(odo)

    def switch_state(self, action):
        self.update_robot(action)
        state = {
            "stop": WalkingStateMachine.waiting,
            "linear_set_target": WalkingStateMachine.linear_go_to,
            "smart_set_target": WalkingStateMachine.smart_go_to,
            "set_rotation_target": WalkingStateMachine.go_around,
        }.get(str(action), self)
        return state

    def update(self, action):
        state = self.switch_state(action)
        state.update_robot(action)
        state.action = action
        return state

    def backup_autoapply(self):
        self.autoapply_enabled = self.robot.locomotion.autoapply.enable()

    def restore_autoapply(self):
        if self.get_parameters() == [0.0, 0.0, 0.0]:
            self.robot.locomotion.autoapply.enable(self.autoapply_enabled)

    def is_done(self):
        return True

    def look_at(self, x, y, z=0.0):
        joints = self.robot.kinematics.jointsLookAt(x, y, z, False)
        self.robot.locomotion.head.positions(joints[0], joints[1])
        self.robot.locomotion.head.hardness(0.8, 0.8)



class Waiting(WalkingState):
    def run(self):
        pass


class LinearGoTo(WalkingState):
    SLEEP_TIME = 0.5
    POSITION_INACCURACY = 50.0

    def __init__(self, ctx=None, action=None):
        super(LinearGoTo, self).__init__(ctx, action)
        self.iterrupt = False
        self.set_thread_safe_all(("iterrupt",))

    def _worker(self):
        action = None
        self.iterrupt = False
        while not self.iterrupt:
            _action = self.action
            if action != _action:
                action = _action
                self.odo_reset()
                odo = [0.0, 0.0, 0.0]
                reached = False
            else:
                odo = self.get_odo()
            dx = action.x - odo[0]
            dy = action.y + odo[1]
            if math.hypot(dx, dy) > self.POSITION_INACCURACY and not reached:
                angle = math.atan2(dy, dx)
                vx = action.speed * math.cos(angle)
                vy = action.speed * math.sin(angle)
                self.set_parameters(vx, vy, 0.0)
            else:
                if not reached:
                    self.odo_reset()
                    reached = True
                    self.set_parameters(0.0, 0.0, 0.0)
            self.set_autoupdate(True)
            time.sleep(self.SLEEP_TIME)
        self.set_parameters(0.0, 0.0, 0.0)

    def prepare(self, action):
        self.action = action
        self.update_robot(action)
        self.backup_autoapply()
        self.iterrupt = False
        self.worker = Thread(target=LinearGoTo._worker, args=(self,))
        self.worker.start()

    def run(self):
        pass

    def finalize(self):
        self.iterrupt = True
        self.worker.join()
        while not self.robot.locomotion.is_done() \
                and self.get_parameters() == [0.0, 0.0, 0.0]:
            time.sleep(self.SLEEP_TIME)
        self.restore_autoapply()

    def is_done(self):
        return self.robot.locomotion.is_done()# and self.get_parameters() == [0.0, 0.0, 0.0]
    

class SmartGoTo(WalkingState, ThreadSafe):
    SLEEP_TIME = 0.5
    MAX_ANGULAR_SPEED = math.radians(10.0)
    ANGLE_INACCURACY = math.radians(35.0)
    POSITION_INACCURACY = 70.0
    _PI3 = math.pi / 3.0

    def __init__(self, ctx=None, action=None):
        super(SmartGoTo, self).__init__(ctx, action)
        self.iterrupt = False
        self.set_thread_safe_all(("iterrupt",))

    def prepare(self, action):
        self.action = action
        self.update_robot(action)
        self.backup_autoapply()
        self.iterrupt = False
        self.worker = Thread(target=SmartGoTo._worker, args=(self,))
        self.worker.start()

    def _worker(self):
        action = None
        q = None
        self.odo_reset()
        state = _WalkState.SO_MUCH_ROTATE
        self.iterrupt = False
        update_action = False;
        while not self.iterrupt:
            # Update speed and target. Or action.
            if action != self.action or update_action:
                q = None
                reached = False
                action = self.action
                r = action.speed / self.MAX_ANGULAR_SPEED
                angle = math.atan2(action.y, action.x)
                self.odo_reset()
                odo = [0.0, 0.0, 0.0]
                update_q = True
                state = _WalkState.SO_MUCH_ROTATE
                update_action = False
            else:
                odo = self.get_odo()
            if reached:
                self.set_parameters(0.0, 0.0, 0.0)
                time.sleep(self.SLEEP_TIME)
                continue
            a = angle - odo[2]
            dist_left = (action.x - odo[0],
                         action.y + odo[1])
            if q is None or update_q:
                q_target = (action.x, action.y)
                sign = (1.0, -1.0)
                angle_to_p = tuple(odo[2] - math.pi / 2.0 * s for s in sign)
                p = tuple((odo[0] + r * math.cos(angle),
                          (odo[1] + r * math.sin(angle))) for angle in angle_to_p)
                d = tuple((q_target[0] - point[0], q_target[1] - point[1]) for point in p)
                h = tuple(math.hypot(d_i[0], d_i[1]) for d_i in d)
                index, _ = min(enumerate(h), key=itemgetter(1))
                # print 'r', r, 'h', h[index]
                if r < h[index]:
                    phi = math.atan2(d[index][1], d[index][0])
                    q_theta = math.acos(r / h[index])
                    q = (p[index][0] + r * math.cos(phi + q_theta * sign[index]),
                        (p[index][1] + r * math.sin(phi + q_theta * sign[index])))
                    theta = -sign[index] * self.MAX_ANGULAR_SPEED
                    update_q = False
            # Rotate at an angle greater than PI / 3
            # print 'state = ', str(state), ', q = ', q, ', odo = ', odo, ', target = ', action.x, action.y
            if any(state == s for s in (_WalkState.SO_MUCH_ROTATE, _WalkState.MUCH_ROTATE)):
                abbabbbebe = math.copysign(1.0, angle) # It's rotate direction. We haven't any ideas about its name :D
                if abs(odo[2]) >= self._PI3:
                    if state == _WalkState.MUCH_ROTATE:
                        state = _WalkState.ROTATE
                    else:
                        state = _WalkState.MUCH_ROTATE
                    c = math.cos(-self._PI3 * abbabbbebe)
                    s = math.sin(-self._PI3 * abbabbbebe)
                    self.action.x, self.action.y = action.x * c - action.y * s, action.x * s + action.y * c
                    update_action = True
                    continue
                if angle < self._PI3:
                    state = _WalkState.ROTATE
                    continue
                elif angle < self._PI3 * 2.0 and state != _WalkState.MUCH_ROTATE:
                    state = _WalkState.MUCH_ROTATE
                    continue
                # Speed update
                self.set_parameters(0.0, 0.0, self.MAX_ANGULAR_SPEED * abbabbbebe)
            # Rotate to a target
            elif state == _WalkState.ROTATE:
                if q and abs(a) < self.ANGLE_INACCURACY:
                    state = _WalkState.GO_CURVE
                    continue
                # elif abs(a) < math.degrees(7):
                #     state = _WalkState.GO_STRAIGHT
                #     continue
                update_q = True
                # Speed update
                self.set_parameters(0.0, 0.0, math.copysign(self.MAX_ANGULAR_SPEED, a))
            # Go to target along curve
            elif state == _WalkState.GO_CURVE:
                # Target reached check
                if math.hypot(dist_left[0], dist_left[1]) < self.POSITION_INACCURACY or \
                                math.hypot(odo[0], odo[1]) >= math.hypot(action.x, action.y):
                    reached = True
                    self.odo_reset()
                    time.sleep(self.SLEEP_TIME)
                    continue
                # Q reached check
                dist_left = (q[0] - odo[0], q[1] + odo[1])
                if math.hypot(dist_left[0], dist_left[1]) < self.POSITION_INACCURACY or abs(a) < math.radians(10.0):
                    state = _WalkState.GO_STRAIGHT
                    continue
                # Speed update
                self.set_parameters(action.speed, 0.0, theta)
            # Go straight
            else:
                if math.hypot(dist_left[0], dist_left[1]) < self.POSITION_INACCURACY or \
                                math.hypot(odo[0], odo[1]) >= math.hypot(action.x, action.y):
                    reached = True
                    self.odo_reset()
                    continue
                # Speed update
                self.set_parameters(action.speed, 0.0, 0.0)
            self.set_autoupdate(True)
            time.sleep(self.SLEEP_TIME)
        self.set_parameters(0.0, 0.0, 0.0)

    def finalize(self):
        self.iterrupt = True
        self.worker.join()
        while not self.robot.locomotion.is_done() \
                and self.get_parameters() == [0.0, 0.0, 0.0]:
            time.sleep(self.SLEEP_TIME)
        self.restore_autoapply()

    def run(self):
        pass

    def is_done(self):
        return self.robot.locomotion.is_done() and self.get_parameters() == [0.0, 0.0, 0.0]
    
    
class GoAround(WalkingState):
    SLEEP_TIME = 0.2
    POSITION_INACCURACY = 50.0
    MAX_ANGULAR_SPEED = math.radians(10.0)
    ANGLE_INACCURACY = math.radians(15.0)

    def __init__(self, ctx=None, action=None):
        super(GoAround, self).__init__(ctx, action)
        self.iterrupt = False
        self.set_thread_safe_all(("iterrupt",))

    def _worker(self):
        action = None
        self.iterrupt = False
        angl = math.radians(5.0)
        while not self.iterrupt:
            _action = self.action
            if action != _action:
                action = _action
                self.odo_reset()
                odo = [0.0, 0.0, 0.0]
                angle = action.angle
                rotdir = math.copysign(1.0, action.angle)
                finished = False
            else:
                odo = self.get_odo()
            if finished:
                self.set_parameters(0.0, 0.0, 0.0)
                time.sleep(self.SLEEP_TIME)
                continue
            if abs(odo[2]) > math.pi / 3:
                self.odo_reset()
                odo = [0.0, 0.0, 0.0]
                angle -= math.pi / 3 * rotdir
            if abs(angle - odo[2]) < angl:
                finished = True
                continue
            self.set_parameters(0.0, 0.0, self.MAX_ANGULAR_SPEED * rotdir * action.scale)
            self.set_autoupdate(True)
            time.sleep(self.SLEEP_TIME)
        self.set_parameters(0.0, 0.0, 0.0)

    def prepare(self, action):
        self.action = action
        self.update_robot(action)
        self.backup_autoapply()
        self.iterrupt = False
        self.worker = Thread(target=GoAround._worker, args=(self,))
        self.worker.start()

    def run(self):
        pass

    def finalize(self):
        self.iterrupt = True
        if self.worker is not None:
            self.worker.join()
        while not self.robot.locomotion.is_done() \
                and self.get_parameters() == [0.0, 0.0, 0.0]:
            time.sleep(self.SLEEP_TIME)
        self.restore_autoapply()

    def is_done(self):
        return self.robot.locomotion.is_done() and self.get_parameters() == [0.0, 0.0, 0.0]


class WalkingStateMachine(StateMachine):
    def __init__(self, walker):
        StateMachine.__init__(self, Waiting(action=WalkCmd.stop(robot=walker.robot)))
        
WalkingStateMachine.waiting = Waiting()
WalkingStateMachine.linear_go_to = LinearGoTo()
WalkingStateMachine.smart_go_to = SmartGoTo()
WalkingStateMachine.go_around = GoAround()


class OdoListener:
    def notify(self, frodo):
        self.odo = frodo


class Walker:
    odo_listeners = []

    def __init__(self, robot):
        self.robot = robot
        self.sm = WalkingStateMachine(walker=self)
        self.action = self.sm.current_state.action

    def stop(self):
        self.action = WalkCmd.stop(robot=self.robot)
        self.update_state_machine()

    def smart_go_to(self, x, y, speed):
        self.action = WalkCmd.smart_go_to(robot=self.robot, x=x, y=y, speed=speed)
        self.update_state_machine()

    def linear_go_to(self, x, y, speed):
        self.action = WalkCmd.linear_go_to(robot=self.robot, x=x, y=y, speed=speed)
        self.update_state_machine()

    def go_around(self, angle, scale=1.0):
        self.action = WalkCmd.go_around(robot=self.robot, angle=angle, scale=scale)
        self.update_state_machine()

    def update_state_machine(self):
        self.sm.update(self.action)
        self.sm.current_state.ctx = Walker.odo_listeners

    def is_done(self):
        return self.sm.current_state.is_done()

    def look_at(self, x, y, z=0.0):
        self.sm.current_state.look_at(x, y, z)

