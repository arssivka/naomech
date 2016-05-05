from behavior.camera_calibator import CamCalib
from behavior.robot import Robot

robot = Robot("192.168.1.2", "5469")
cam_cal = CamCalib(robot)

cam_cal.calibrateCamera()