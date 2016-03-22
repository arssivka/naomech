import math
import numpy as np
import json
import time

from itertools import islice

from graph import Graph


def load_poses(poses, filename):
    with open(filename) as config:
        p = json.load(config)
        for keys in p:
            poses[keys] = Pose(p[keys])


class PoseHandler:
    def __init__(self, robot, freq=8):
        self.poses = {}
        self.robot = robot
        self.keys = range(len(robot.joints.keys()))
        self.freq = freq
        self.sleep_time = 1.0 / freq
        print self.sleep_time

    def nearest_to(self, pose):
        nearest = None
        dist = None
        for key, value in self.poses.iteritems():
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
        if type(target) is str:
            target = self.poses[target]
        if move_time > 0.0:
            joints = self.robot.joints.positions()['data']
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

    def get_poses(self):
        return self.poses.keys()

class PoseSwitcher:
    def __init__(self, pose_handler):
        self.pose_handler = pose_handler
        self.graph = Graph()
        self.update()
        self.switch_time = 0.3

    def update(self):
        poses = set(self.pose_handler.get_poses())
        vertices = set(self.graph.getVertices())
        to_append = poses - vertices
        for pose in to_append:
            self.graph.addVertex(pose)

    def addTransition(self, begin, destination, time):
        self.graph.addEdge(begin, destination, time)

    def switchTo(self, start, destination):
        path = self.graph.shortestPathDijkstra(start, destination)
        current = self.graph.getVertex(start)
        self.pose_handler.set_pose(current, self.switch_time)
        for dest in islice(path, None):
            time = current.getWeight[dest]
            self.pose_handler.set_pose(dest, time)
            current = dest
        

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