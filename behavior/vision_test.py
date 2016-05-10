import time

import math

import timeit
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

robot = Robot("192.168.1.2", "5469")

pose_handler = PoseHandler(robot, 30)
pose.load_poses(pose_handler, "config/poses.json")
pose_switcher = PoseSwitcher(pose_handler)
pose.load_switches(pose_switcher, "config/switches.json")
walk = Walker(robot)
robot.joints.hardness(0.8)
pose_handler.set_pose('walking_pose', 1.0)

try:
    top_camera = False

    i = 0
    while True:
        raw = robot.cameras.image(top_camera)
        img = camera_calibator.converToCvYUV(raw.get('data').data)
        picturerbg = cv2.cvtColor(img, cv2.COLOR_YUV2RGB)
        begin = timeit.default_timer()

        robot.vision.updateFrame()
        ball = robot.vision.ballDetect()
        lines = robot.vision.lineDetect()
        # cv2.imwrite('/home/tekatod/bases/real_image/' + str(i) + '.png', picturerbg)
        i += 1

        end = timeit.default_timer()
        for line in lines:
            cv2.line(picturerbg, (line['x1'], line['y1']), (line['x2'], line['y2']), (255, 0, 255))
        cv2.rectangle(picturerbg, (ball['x'], ball['y']),
                      (ball['x'] + ball['width'], ball['y'] + ball['height']), (0, 255, 255))
        cv2.imshow('robotVis', picturerbg)

        cv2.waitKey(1000)
        print (end - begin), len(lines), ball

finally:
    pass