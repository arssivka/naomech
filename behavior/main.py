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



robot = Robot("192.168.1.64", "5469")
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
# robot.joints.hardness([0, 1], [0.0, 0.0])
# rleg_keys = ['R_HIP_YAW_PITCH', 'R_HIP_ROLL', 'R_HIP_PITCH', 'R_KNEE_PITCH', 'R_ANKLE_PITCH', 'R_ANKLE_ROLL']
# lleg_keys = ['L_HIP_YAW_PITCH', 'L_HIP_ROLL', 'L_HIP_PITCH', 'L_KNEE_PITCH', 'L_ANKLE_PITCH', 'L_ANKLE_ROLL']
# robot.joints.hardness(lleg_keys, [0.1 for i in lleg_keys])
# robot.joints.hardness(rleg_keys, [0.1 for i in rleg_keys])
# behavior = BehaviorHandler(robot, walk, pose_handler, pose_switcher)

try:
    pass
    # behavior.run()
    # while True:
    #     pass
    # pose_switcher.switch_to("prepare_left_kick", "walking_pose")
    # walk.go_to((-500.0, 000.0), 100.0)
    # while walk.get_speed() != 0.0:
    #     pass
    # pose_switcher.switch_to("prepare_left_kick", "walking pose")
    # walk.go_to((0.0, 1000.0), 100.0)
    # while walk.get_speed() != 0.0:
    #     pass
    # pose_switcher.switch_to("prepare_right_kick", "walking pose")
    # pose_switcher.switch_to("prepare_left_kick", "walking_pose")
    # behavior.run()
    # while True:
    #      pass
    # #TODO: Make the seperate function for this stand up
    # ###here goes  the stand_up
    # start = time.time()
    # pose_handler.set_pose("face_floor", 0.3)
    # pose_handler.set_pose("drunken_sea_star", 0.3)
    # pose_handler.set_pose("arm_spin_1", 0.3)
    # pose_handler.set_pose("arm_spin_2", 0.3)
    # pose_handler.set_pose("koryaka_razebaka", 0.3)
    # pose_handler.set_pose("koryaka_sobiraka", 0.3)
    # pose_handler.set_pose("crabe", 0.5)
    # pose_handler.set_pose("monkey", 1.2)
    # pose_handler.set_pose("after_monkey_1", 1.2)
    # pose_handler.set_pose("after_monkey_2", 1.2)
    # pose_handler.set_pose("after_monkey_3", 0.7)
    # pose_handler.set_pose("after_monkey_4", 0.7)
    # pose_handler.set_pose("after_monkey_5", 0.5)
    # pose_handler.set_pose("sit", 0.5)
    # print time.time() - start
    ###here stand up ends

    #TODO: Make the seperate function for this stand up/ Some messy movements from trying_to_sit to sit
    #####FACE_UP_STANDS_UP_GOES_HERE
    # pose_handler.set_pose("fapose 0.7)

    #TODO: Try ta add "prepare_..._kick"(left to right etc) to the end of each kick motion for the smoothing
    # for i in range(10):
    #     pose_switcher.switchTo("prepare_left_kick", "walking_pose")
    #     time.sleep(1)
    #     pose_switcher.switchTo("prepare_right_kick", "walking_pose")
    # robot.kinematics.lookAt(100.0, 0.0, 0.0, False)
    # iters = 100
    # keys = robot.joints.keys()
    # pose_data = [0.0] * len(keys)
    # for i in range(iters):
    #     joints =  robot.joints.positions()['data']
    #     pose_data = [a + b for a, b in zip(pose_data, joints)]
    #     time.sleep(1.0 / iters)
    # pose_data = [joint / iters for joint in pose_data]
    # print pose_data
    # print robot.locomotion.parameters.keys()
    # robot.locomotion.parameters(['Y', 'STEP_COUNT' ], [100.0, 10.0])
    # robot.locomotion.parameters(['X', 'STEP_COUNT' ], [100.0, 10.0])
    # robot.locomotion.parameters(['X', 'STEP_COUNT' ], [-100.0, 10.0])
    # robot.locomotion.parameters(['Y', 'STEP_COUNT' ], [100.0, 5.0])


    stance = [310.0, 14.5, 100.0, 0.05, 0.0, 0.1]
    step = [0.6, 0.45, 10.0, 0.0, 70.0, -50.0, 70.0, 0.35, 70.0, 70.0, 0.35, 1.0]
    zmp = [0.0, 0.9, 4.0, 4.0, 0.01, 6.6]
    hack = [0.05, 0.05]
    sensor = [0.0,
             0.0,
             0.0,
             0.0,
             0.0,
             0.0,
             0.0,
             0.0]
    stiff = [0.85, 0.3, 0.4, 0.3, 0.2, 0.2]
    odo = [1.0, 1.0, 1.3]
    arm = [0.3]
    robot.locomotion.gait(stance, step, zmp, hack, sensor, stiff, odo, arm)

    pose_handler.set_pose("walking_pose", 1.0)
    robot.kinematics.lookAt(1000.0, 000.0, 0.0, False)
    # robot.kinematics.lookAt(1000.0, 00.0, 0.0, False)
    time.sleep(0.5)
    # while Tr

    # # walk.go_to((480.6, 100.0), 100.0)
    # robot.vision.updateFrame()
    # ball = robot.vision.ballDetect()
    # pix = cg.imagePixelToWorld(ball["x"] + ball["width"]/2, ball["y"], False)
    # robot.kinematics.lookAt(2000.0, 0.0, 0.0, False)
    # time.sleep(3.0)
    # if pix[0] > 0.0 and ball["width"] > 0.0:
    #     while math.hypot(pix[0], pix[1]) > 150.0:
    #         robot.vision.updateFrame()
    #         ball = robot.vision.ballDetect()
    #         pix = cg.imagePixelToWorld(ball["x"] + ball["width"]/2, ball["y"], False)
    #         print pix
    #         if pix[0] > 0.0 and ball["width"] > 0.0:
    #             joints = robot.kinematics.jointsLookAt(pix[0], pix[1], 0.0, False)
    #             robot.locomotion.head.hardness(0.8, 0.8)
    #             robot.locomotion.head.positions(joints[0], joints[1])
    #             walk.go_to(pix, 100.0)
    #             time.sleep(1.0)
    #     walk.stop()
    # robot.locomotion.autoapply.enable(False)
    # robot.joints.hardness(0.85)
    # if pix[1] < 0.0:
    #     pose_switcher.switch_to("prepare_right_kick", "walking_pose")
    # else:
    #     pose_switcher.switch_to("prepare_left_kcik", "walking_pose")
    #

    # robot.locomotion.autoapply.enable(False)
    # robot.locomotion.odo3etry(True)
    # print robot.locomotion.odometry()
    # robot.locomotion.odometry(True)
    # print robot.locomotion.odometry()
    #

    # camc = CamCalib(robot)
    # camc.calibrateCamera(True)
    # cam.imagePixelToWorld(50, 50, False)
    # walk = Walker(robot)
    # print walk.get_speed()
    # walk.go_to((000.0, 500.0), 100.0)

    # walk.go_around((200.0, 0.0), 20.0)
    # walk.set_speed(0.0, 0.0, 0.0)
    # start = time.time()
    # time.sleep(10.0)
    # print robot.locomotion.odometry()['data']
    loc = LocaTesting(robot, cg)
    loc.get_sensors()
    loc.print_plot()
    loc = LocalizationModule(robot, cg)
    loc.localization()
    print loc.position.point
    loc.print_plot(once = True)
    # point = loc.global_to_local(loc.map.friendly_point.x + 200, 0)
    # loc.print_plot(once=True)
    # robot.kinematics.lookAt(0.0, 000.0, 0.0, False)
    # print point
    # walk.go_to(point,100.0)


    # robot.vision.updateFrame()
    # ball = robot.vision.ballDetect()
    # pix = cam.imagePixelToWorld(ball["x"] + ball["width"]/2, ball["y"], False)
    # walk = Walker(robot)
    # joints = robot.kinematics.jointsLookAt(pix[0], pix[1], 0.0, False)
    # robot.locomotion.head.positions(joints[0], joints[1])
    # robot.locomotion.head.hardness(0.8, 0.8)
    # count = 0
    # while math.hypot(pix[0], pix[1] > 300):
    #         robot.vision.updateFrame()
    #         ball = robot.vision.ballDetect()
    #         pix = cam.imagePixelToWorld(ball["x"] + ball["width"]/2, ball["y"], False)
    #         count += 1
    #         if pix is not None and pix[0] > 0:
    #             if count == 49:
    #                 joints = robot.kinematics.jointsLookAt(pix[0], pix[1], 0.0, False)
    #                 robot.locomotion.head.positions(joints[0], joints[1])
    #                 robot.locomotion.head.hardness(0.8, 0.8)
    #                 count = 0
    #             print pix
    #             walk.go_to((pix[0], pix[1]), 100.0)
    # walk.stop()

    # iters = 100
    # keys = robot.joints.keys()
    # pose_data = [0.0] * len(keys)
    # for i in range(iters):
    #     joints =  robot.joints.positions()['data']
    #     pose_data = [a + b for a, b in zip(pose_data, joints)]
    #     time.sleep(1.0 / iters)
    # pose_data = [joint / iters for joint in pose_data]
    # print pose_data
finally:
    pass
    # behavior.stop()
    # robot.joints.hardness(0.0)