import math
import numpy as np
import json
import time

from itertools import islice, izip
from threading import Lock
from graph import Graph


def load_poses(pose_handler, filename):
    with open(filename) as config:
        data = json.load(config)
    poses = {key: Pose(positions) for (key, positions) in data.items()}
    pose_handler.add_poses(poses)

class PoseHandler:
    def __init__(self, robot, freq=64):
        self.poses = {}
        self.robot = robot
        self.keys = range(len(robot.joints.keys()))
        self.freq = freq
        self.sleep_time = 1.0 / freq
        self._lock = Lock()

    def set_sreq(self, freq):
        self._lock.acquire()
        try:
            self.sleep_time = 1.0 / freq
        finally:
            self._lock.release()

    def add_poses(self, poses):
        self._lock.acquire()
        try:
            self.poses.update(poses)
        finally:
            self._lock.release()

    def nearest_to(self, pose):
        nearest = None
        dist = None
        self._lock.acquire()
        try:
            poses = self.poses.copy()
        finally:
            self._lock.release()
        for key, value in poses.iteritems():
            if nearest is None:
                nearest = (key, value)
                dist = pose - value
                continue
            new_dist = pose - value
            if abs(new_dist) < abs(dist):
                dist = new_dist
                nearest = (key, value)
        return nearest

    def set_pose(self, target, move_time=0.0):
        if not isinstance(target, Pose):
            self._lock.acquire()
            try:
                target = self.poses[target]
            finally:
                self._lock.release()
        if move_time > 0.0:
            joints = self.robot.joints.positions().get('data')
            initial = Pose(joints)
            count = move_time / self.sleep_time
            dpose = (target - initial) / count
            for i in range(int(count)):
                start = time.time()
                initial += dpose
                self.robot.joints.positions(self.keys, initial.positions.tolist())
                end = time.time()
                sleep = self.sleep_time - (end - start)
                if sleep > 0.0:
                    time.sleep(sleep)
        else:
            self.robot.joints.positions(self.keys, target.positions.tolist())

    def get_pose_keys(self):
        self._lock.acquire()
        try:
            keys = self.poses.keys()
        finally:
            self._lock.release()
        return keys


def load_switches(switcher, filename):
    switcher.update()
    with open(filename) as config:
        p = json.load(config)
    for key, motion in p.items():
        current = motion[0][0]
        for target, time in islice(motion, 1, None):
            switcher.add_transition(current, target, time)
            current = target


class PoseSwitcher:
    def __init__(self, pose_handler):
        self.pose_handler = pose_handler
        self.graph = Graph()
        self._lock = Lock()
        self.update()
        self.switch_time = 0.3
        self._complete = 1.0
        self._stop = False

    def is_done(self):
        self._lock.acquire()
        try:
            complete = self._complete
        finally:
            self._lock.release()
        return complete >= 1.0

    def stop(self):
        self._lock.acquire()
        try:
            self._stop = True
        finally:
            self._lock.release()

    def update(self):
        poses = set(self.pose_handler.get_pose_keys())
        vertices = set(self.graph.getVertices())
        to_append = poses - vertices
        for pose in to_append:
            self._lock.acquire()
            try:
                self.graph.addVertex(pose)
            finally:
                self._lock.release()

    def add_transition(self, begin, destination, time):
        self._lock.acquire()
        try:
            self.graph.addEdge(begin, destination, time)
        finally:
            self._lock.release()

    def switch_to(self, start, destination):
        delays = []
        self._lock.acquire()
        try:
            self._stop = False
            self._complete = 0.0
            path = self.graph.shortestPathDijkstra(start, destination)
            current = self.graph.getVertex(start)
            for dest in islice(path, 1, None):
                dt = current.getWeight(dest)
                delays.append(dt)
                current = self.graph.getVertex(dest)
            switch_time = self.switch_time
        finally:
            self._lock.release()
        total = switch_time + sum(delays)
        self.pose_handler.set_pose(start, self.switch_time)
        self._complete = switch_time / total
        self._lock.acquire()
        try:
            self._complete = switch_time / total
        finally:
            self._lock.release()
        for target, delay in izip(islice(path, 1, None), delays):
            self.pose_handler.set_pose(target, delay)
            self._lock.acquire()
            try:
                self._complete = dt / total
                if self._stop:
                    break
            finally:
                self._lock.release()
            self._complete += dt / total
        

class Pose:
    def __init__(self, positions):
        self.positions = np.array(positions)
        np.power(positions, 2)

    def __str__(self):
        return str(self.positions)

    def __mul__(self, other):
        try:
            return Pose(self.positions * other.positions)
        except AttributeError:
            return Pose(self.positions * other)

    def __div__(self, other):
        try:
            return Pose(self.positions / other.positions)
        except AttributeError:
            return Pose(self.positions / other)

    def __add__(self, other):
        try:
            return Pose(self.positions + other.positions)
        except AttributeError:
            return Pose(self.positions)

    def __sub__(self, other):
        try:
            return Pose(self.positions - other.positions)
        except AttributeError:
            return Pose(self.positions - other)

    def __abs__(self):
        return Pose(math.sqrt(np.sum(np.power(self.positions, 2))))