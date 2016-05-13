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



robot = Robot("192.168.1.3", "5469")
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

stance = [310.0, 14.5, 100.0, 0.0, 0.0, 0.1]
step = [0.8, 0.30, 15.0, 0.0, 70.0, -50.0, 70.0, 0.35, 70.0, 70.0, 0.35, 1.0]
zmp = [0.0, 0.0, 30.0, 30.0, 0.01, 1.6]
hack = [0.0, 0.0]
sensor = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
stiff = [0.85, 0.7, 0.4, 0.3, 0.2, 0.2]
odo = [1.0, 1.0, 1.5]
arm = [0.3]
robot.locomotion.gait(stance, step, zmp, hack, sensor, stiff, odo, arm)


def localize():
    joint_keys = ["HEAD_PITCH", "HEAD_YAW", "L_SHOULDER_PITCH", "L_SHOULDER_ROLL", "L_ELBOW_YAW", "L_ELBOW_ROLL",
                  "L_WRIST_YAW", "L_HAND", "L_HIP_YAW_PITCH", "L_HIP_ROLL", "L_HIP_PITCH", "L_KNEE_PITCH",
                  "L_ANKLE_PITCH", "L_ANKLE_ROLL", "R_HIP_YAW_PITCH", "R_HIP_ROLL", "R_HIP_PITCH", "R_KNEE_PITCH",
                  "R_ANKLE_PITCH", "R_ANKLE_ROLL", "R_SHOULDER_PITCH", "R_SHOULDER_ROLL", "R_ELBOW_YAW", "R_ELBOW_ROLL",
                  "R_WRIST_YAW", "R_HAND"]
    hardness_values = [0.0, 0.0] + [0.8] * (len(joint_keys) - 2)
    robot.joints.hardness(joint_keys, hardness_values)
    pose_handler.set_pose("walking_pose", 2.0)
    leds_keys = ["L_FACE_RED_0_DEG", "L_FACE_GREEN_0_DEG", "L_FACE_BLUE_0_DEG", "L_FACE_RED_135_DEG",
                 "L_FACE_GREEN_135_DEG", "L_FACE_BLUE_135_DEG", "L_FACE_RED_180_DEG", "L_FACE_GREEN_180_DEG",
                 "L_FACE_BLUE_180_DEG", "L_FACE_RED_225_DEG", "L_FACE_GREEN_225_DEG", "L_FACE_BLUE_225_DEG",
                 "L_FACE_RED_270_DEG", "L_FACE_GREEN_270_DEG", "L_FACE_BLUE_270_DEG", "L_FACE_RED_315_DEG",
                 "L_FACE_GREEN_315_DEG", "L_FACE_BLUE_315_DEG", "L_FACE_RED_45_DEG", "L_FACE_GREEN_45_DEG",
                 "L_FACE_BLUE_45_DEG", "L_FACE_RED_90_DEG", "L_FACE_GREEN_90_DEG", "L_FACE_BLUE_90_DEG",
                 "R_FACE_RED_0_DEG", "R_FACE_GREEN_0_DEG", "R_FACE_BLUE_0_DEG", "R_FACE_RED_135_DEG",
                 "R_FACE_GREEN_135_DEG", "R_FACE_BLUE_135_DEG", "R_FACE_RED_180_DEG", "R_FACE_GREEN_180_DEG",
                 "R_FACE_BLUE_180_DEG", "R_FACE_RED_225_DEG", "R_FACE_GREEN_225_DEG", "R_FACE_BLUE_225_DEG",
                 "R_FACE_RED_270_DEG", "R_FACE_GREEN_270_DEG", "R_FACE_BLUE_270_DEG", "R_FACE_RED_315_DEG",
                 "R_FACE_GREEN_315_DEG", "R_FACE_BLUE_315_DEG", "R_FACE_RED_45_DEG", "R_FACE_GREEN_45_DEG",
                 "R_FACE_BLUE_45_DEG", "R_FACE_RED_90_DEG", "R_FACE_GREEN_90_DEG", "R_FACE_BLUE_90_DEG"]
    leds_on = [1.0, 0.0, 0.0] * (len(leds_keys) / 3)
    leds_off = [0.0] * len(leds_keys)
    robot.leds.brightness(leds_keys, leds_on)
    time.sleep(2.5)
    robot.leds.brightness(leds_keys, leds_off)
    loc.localization()
    print loc.position.point


try:
    print "localization started"
    # localize()
    print "localization ended"
    # walk.linear_go_to(00, 300, 100)
    behavior = BehaviorHandler(robot, walk, pose_handler, pose_switcher, cg, loc)
    walk.odo_listeners.append(behavior)
    behavior.run()
    # walk.smart_go_to(500.0, 300.0, 100.0)
finally:
    pass
    behavior.stop()
    # robot.joints.hardness(0.0)