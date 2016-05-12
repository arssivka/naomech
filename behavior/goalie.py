import time

import math

import pose
from behavior import GoalieBehaviourHandler
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

stance = [310.0, 14.5, 100.0, 0.0, 0.0, 0.1]
step = [0.8, 0.30, 15.0, 0.0, 70.0, -50.0, 70.0, 0.35, 70.0, 70.0, 0.35, 1.0]
zmp = [0.0, 0.0, 30.0, 30.0, 0.01, 1.6]
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
def ball_recognitor():
    robot.vision.updateFrame()
    ball = robot.vision.ballDetect()
    pix = cg.imagePixelToWorld(ball["x"] + ball["width"]/2, ball["y"], False)
    return ball["width"], pix

def ball_finder():
    robot.locomotion.autoapply.enable(False)
    ball_w, pix = ball_recognitor()
    la_coords = [1000.0, 0.0]
    step = 100.0
    if ball_w == 0:
        while ball_w == 0:
            la_coords[1] += step
            robot.kinematics.lookAt(la_coords[0], la_coords[1], 0.0, False)
            time.sleep(0.5)
            ball_w, pix = ball_recognitor()
            if abs(la_coords[1]) >= 1000.0:
                step *= -1
                pose_handler.set_pose("walking_pose", 1.0)
    robot.kinematics.lookAt(pix[0], pix[1], 0.0, False)

def ball_blocker():
    ball_w, pix = ball_recognitor()
    while abs(pix[1]) > 100:
        ball_w, pix = ball_recognitor()
        if pix is not None and pix[0] > 0 and ball_w > 0:
            joints = robot.kinematics.jointsLookAt(pix[0], pix[1], 0.0, False)
            robot.locomotion.head.positions(joints[0], joints[1])
            robot.locomotion.head.hardness(0.8, 0.8)
            tmp_p = loc.position.point.y + pix[1]
            if tmp_p > 1100.0 or tmp_p < -1100.0:
                walk.linear_go_to(0.0, math.copysign((1100.0 - abs(tmp_p)), pix[1]), 100.0)
            else:
                walk.linear_go_to(0.0, pix[1], 100.0)
            time.sleep(0.5)
    walk.stop()
    robot.locomotion.autoapply.enable(False)

def go_to_posuture():
    tuc = loc.global_to_local(loc.map.start_point.x, loc.map.start_point.y)
    while math.hypot(tuc[0], tuc[1]) > 100:
        tuc = loc.global_to_local(loc.map.start_point.x, loc.map.start_point.y)
        walk.smart_go_to(tuc[0], tuc[1], 100.0)
        joints = robot.kinematics.jointsLookAt(500.0, 0.0, 0.0, False)
        robot.locomotion.head.positions(joints[0], joints[1])
        robot.locomotion.head.hardness(0.8, 0.8)
        time.sleep(2.0)
    walk.stop()
    tuc = loc.global_to_local(loc.map.start_point.x, loc.map.start_point.y)
    walk.go_around(tuc[2])
    robot.locomotion.autoapply.enable(False)

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
    time.sleep(5.0)
    robot.leds.brightness(leds_keys, leds_off)
    loc.localization()
    print loc.position.point

try:
    print "localization started"
    localize()
    print "localization ended"
    go_to_posuture()
    behavior = GoalieBehaviourHandler(robot, walk, pose_handler, pose_switcher, cg, loc)

finally:
    pass
    behavior.stop()
    # robot.joints.hardness(0.0)