from behavior import pose
from behavior.robot import Robot
from behavior.camera_geometry import CamGeom
from behavior.pose import PoseHandler
from behavior.pose import PoseSwitcher
from behavior.walker import Walker
import time
import math

robot = Robot("192.168.1.3", "5469")
cg = CamGeom("../config/cameras.json", robot)
pose_handler = PoseHandler(robot, 30)
pose.load_poses(pose_handler, "../config/poses.json")
pose_switcher = PoseSwitcher(pose_handler)
pose.load_switches(pose_switcher, "../config/switches.json")
robot.joints.hardness(0.8)
pose_handler.set_pose("walking_pose", 1.0)
walk = Walker(robot)

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
    robot.kinematics.lookAt(pix[0], pix[1], 0.0, False)

def ball_kicker():
    ball_w, pix = ball_recognitor()
    while math.hypot(pix[0], pix[1]) > 400:
            ball_w, pix = ball_recognitor()
            if pix is not None and pix[0] > 0 and ball_w > 0:
                joints = robot.kinematics.jointsLookAt(pix[0], pix[1], 0.0, False)
                robot.locomotion.head.positions(joints[0], joints[1])
                robot.locomotion.head.hardness(0.8, 0.8)
                walk.smart_go_to(pix[0], pix[1], 100.0)
                time.sleep(0.5)
    walk.stop()
    robot.locomotion.autoapply.enable(False)
    ball_w, pix = ball_recognitor()
    if pix[1] > 0:
        pose_switcher.switch_to("prepare_left_kick", "walking_pose")
    else:
        pose_switcher.switch_to("prepare_right_kick", "walking_pose")

ball_finder()
ball_kicker()