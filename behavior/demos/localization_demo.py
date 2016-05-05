from behavior import pose
from behavior.pose import PoseHandler
from behavior.pose import PoseSwitcher
from behavior.robot import Robot
from behavior.camera_geometry import CamGeom
from behavior.localization import LocalizationModule
from behavior.localization import LocaTesting

robot = Robot("192.168.1.2", "5469")
cg = CamGeom("../config/cameras.json", robot)
pose_handler = PoseHandler(robot, 30)
pose.load_poses(pose_handler, "../config/poses.json")
pose_switcher = PoseSwitcher(pose_handler)
pose.load_switches(pose_switcher, "../config/switches.json")

robot.joints.hardness(0.8)
pose_handler.set_pose("walking_pose", 1.0)
robot.joints.hardness([0, 1], [0.0, 0.0])

loctest = LocaTesting(robot, cg)
loctest.get_sensors()
loctest.print_plot()

loc = LocalizationModule(robot, cg)
loc.localization()
loc.print_plot(once=True)