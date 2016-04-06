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

import cv2
from robot import Robot
from camera_geometry import CamGeom
import camera_calibator
import time
import numpy as np

robot = Robot("192.168.0.29", "5469")
print robot.system.listMethods()
# print len(robot.joints.keys())

pose_handler = PoseHandler(robot, 30)
pose.load_poses(pose_handler, "config/poses.json")
pose_switcher = PoseSwitcher(pose_handler)
pose.load_switches(pose_switcher, "config/switches.json")
walk = Walker(robot)
# print robot.joints.keys()
robot.joints.hardness(0.8)
pose_handler.set_pose('walking_pose', 1.0)
# rleg_keys = ['R_HIP_YAW_PITCH', 'R_HIP_ROLL', 'R_HIP_PITCH', 'R_KNEE_PITCH', 'R_ANKLE_PITCH', 'R_ANKLE_ROLL']
# lleg_keys = ['L_HIP_YAW_PITCH', 'L_HIP_ROLL', 'L_HIP_PITCH', 'L_KNEE_PITCH', 'L_ANKLE_PITCH', 'L_ANKLE_ROLL']
# robot.joints.hardness(lleg_keys, [0.1 for i in lleg_keys])
# robot.joints.hardness(rleg_keys, [0.1 for i in rleg_keys])



try:
    top_camera = False
    raw = robot.cameras.image(top_camera)
    img = camera_calibator.converToCvYUV(raw.get('data').data)
    picturerbg = cv2.cvtColor(img, cv2.COLOR_YUV2RGB)
    cv2.imshow('robotVis', picturerbg)
    cv2.imwrite('/home/nikitas/img.png', picturerbg)
    robot.vision.updateFrame()
    ball = robot.vision.ballDetect()
    lines = robot.vision.lineDetect()
    print ball
    print lines
    cv2.waitKey()


finally:
    pass