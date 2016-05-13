from behavior import pose
from behavior.pose import PoseHandler
from behavior.pose import PoseSwitcher
from behavior.robot import Robot
from behavior.camera_geometry import CamGeom
from behavior.localization import LocalizationModule
from behavior.localization import LocaTesting
from behavior.walker import Walker
import math
import time

robot = Robot("192.168.1.3", "5469")
cg = CamGeom("../config/cameras.json", robot)
pose_handler = PoseHandler(robot, 30)
pose.load_poses(pose_handler, "../config/poses.json")
pose_switcher = PoseSwitcher(pose_handler)
pose.load_switches(pose_switcher, "../config/switches.json")
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

robot.joints.hardness(0.8)
pose_handler.set_pose("walking_pose", 1.0)
robot.joints.hardness([0, 1], [0.0, 0.0])

loctest = LocaTesting(robot, cg)
loctest.get_sensors()
loctest.print_plot()

loc = LocalizationModule(robot, cg)
loc.localization()
loc.print_plot(once=True)


tuc = loc.global_to_local(loc.map.start_point.x, loc.map.start_point.y)
print tuc
walk.odo_listeners.append(loc)
while math.hypot(tuc[0], tuc[1]) > 100:
    tuc = loc.global_to_local(loc.map.start_point.x, loc.map.start_point.y)
    print tuc
    walk.smart_go_to(tuc[0], tuc[1], 100.0)
    joints = robot.kinematics.jointsLookAt(500.0, 0.0, 0.0, False)
    robot.locomotion.head.positions(joints[0], joints[1])
    robot.locomotion.head.hardness(0.8, 0.8)
    time.sleep(2.0)
walk.stop()
robot.locomotion.autoapply.enable(False)
loc.print_plot(once=True)
