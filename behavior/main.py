import time

import math

import pose
from behavior import BehaviorHandler
from pose import PoseHandler
from pose import PoseSwitcher
from robot import Robot
from walker import Walker
from camera_calibator import CamCalib
from camera_geometry import CamGeom
from localization import LocalizationModule
from localization import LocaTesting



robot = Robot("192.168.1.2", "5469")
print robot.system.listMethods()
print len(robot.joints.keys())

pose_handler = PoseHandler(robot, 30)
pose.load_poses(pose_handler, "config/poses.json")
pose_switcher = PoseSwitcher(pose_handler)
pose.load_switches(pose_switcher, "config/switches.json")
cg = CamGeom("config/cameras.json", robot)
walk = Walker(robot)
print robot.joints.keys()
robot.joints.hardness(0.8)
loc = LocalizationModule(robot, cg)
walk.odo_listeners.append(loc)
# robot.joints.hardness([0, 1], [0.0, 0.0])
# rleg_keys = ['R_HIP_YAW_PITCH', 'R_HIP_ROLL', 'R_HIP_PITCH', 'R_KNEE_PITCH', 'R_ANKLE_PITCH', 'R_ANKLE_ROLL']
# lleg_keys = ['L_HIP_YAW_PITCH', 'L_HIP_ROLL', 'L_HIP_PITCH', 'L_KNEE_PITCH', 'L_ANKLE_PITCH', 'L_ANKLE_ROLL']
# robot.joints.hardness(lleg_keys, [0.1 for i in lleg_keys])
# robot.joints.hardness(rleg_keys, [0.1 for i in rleg_keys])
behavior = BehaviorHandler(robot, walk, pose_handler, pose_switcher, cg, loc)

stance = [310.0, 14.5, 100.0, 0.0, 0.0, 0.1]
step = [0.75, 0.50, 15.0, 0.0, 70.0, -50.0, 70.0, 0.35, 70.0, 70.0, 0.35, 1.0]
zmp = [0.0, 0.0, 20.0, 20.0, 0.01, 1.6]
hack = [0.0, 0.0]
sensor = [0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0]
stiff = [0.85, 0.7, 0.4, 0.3, 0.2, 0.2]
odo = [1.0, 1.0, 1.5]
arm = [0.3]
robot.locomotion.gait(stance, step, zmp, hack, sensor, stiff, odo, arm)

try:
    pass
    # walk.go_around(math.radians(180.0))
    behavior.run()
finally:
    pass
    behavior.stop()
    # robot.joints.hardness(0.0)