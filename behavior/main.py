import time

import math

import pose
from pose import PoseHandler
from pose import PoseSwitcher
from robot import Robot
from walker import Walker
from camera_calibator import CamCalib
from camera_geometry import CamGeom


robot = Robot("192.168.0.51", "5469")
print robot.system.listMethods()
print len(robot.joints.keys())

pose_handler = PoseHandler(robot, 30)
pose_switcher = PoseSwitcher(pose_handler)
pose.load_poses(pose_handler.poses, "config/poses.json")
pose.load_switches(pose_switcher, "config/switches.json")
print robot.joints.keys()
robot.joints.hardness(0.8)
rleg_keys = ['R_HIP_YAW_PITCH', 'R_HIP_ROLL', 'R_HIP_PITCH', 'R_KNEE_PITCH', 'R_ANKLE_PITCH', 'R_ANKLE_ROLL']
lleg_keys = ['L_HIP_YAW_PITCH', 'L_HIP_ROLL', 'L_HIP_PITCH', 'L_KNEE_PITCH', 'L_ANKLE_PITCH', 'L_ANKLE_ROLL']
# robot.joints.hardness(lleg_keys, [0.1 for i in lleg_keys])
# robot.joints.hardness(rleg_keys, [0.1 for i in rleg_keys])

try:
    pass
    # for pose in [ "face_floor", "drunken_sea_star", "arm_spin_1", "arm_spin_2", "koryaka_razebaka",
    #               "koryaka_sobiraka", "crabe", "monkey", "after_monkey_1", "after_monkey_2", "after_monkey_3", "after_monkey_4", "after_monkey_5", "sit"]:
    #      pose_handler.set_pose(pose, 2.0)
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
    # pose_handler.set_pose("face_up_init", 0.3)
    # pose_handler.set_pose("legs_prepare", 0
    # .3)
    # pose_handler.set_pose("legs_push", 0.3)
    # pose_handler.set_pose("arms_under", 0.3)
    # pose_handler.set_pose("legs_stright", 0.3)
    # pose_handler.set_pose("sitting_1", 0.3)
    # pose_handler.set_pose("sitting_2", 0.3)
    # pose_handler.set_pose("open_the_door", 0.3)
    # pose_handler.set_pose("legs_bend", 0.3)
    # pose_handler.set_pose("starting_standing", 0.3)
    # pose_handler.set_pose("ts_prepare", 0.3)
    # pose_handler.set_pose("trying_to_sit", 0.3)
    # pose_handler.set_pose("pre_sit", 0.3)
    # pose_handler.set_pose("sit", 0.3)
    #
    # pose_handler.set_pose("walking_pose", 0.5)
    # pose_handler.set_pose("prepare_right_kick", 1.0)
    # pose_handler.set_pose("prepare_right_kick_2", 1.0)
    # pose_handler.set_pose("prepare_right_kick_3", 1.0)
    # pose_handler.set_pose("right_leg_up", 1.0)
    # pose_handler.set_pose("right_kick", 1.0)
    # pose_handler.set_pose("after_right_kick_2", 1.0)
    # pose_handler.set_pose("walking_pose", 0.5)
    # temp =  [
    #     -0.1150613594055176,
    #     0.0026271724700927734,
    #     1.6084797906875608,
    #     0.10120201110839845,
    #     -0.00010332107543945313,
    #     -0.08669051170349121,
    #     0.024532718658447264,
    #     0.14918796420097352,
    #     -0.002795910835266113,
    #     0.09854010105133057,
    #     -0.36624658584594727,
    #     0.9753213453292847,
    #     -0.49687383651733397,
    #     -0.1534653615951538,
    #     0.0,
    #     0.1380713748931885,
    #     -0.44250892639160155,
    #     1.0769406986236572,
    #     -0.5445740270614624,
    #     -0.27593992710113524,
    #     1.6261126232147216,
    #     -0.1381020545959473,
    #     -0.0014685678482055662,
    #     0.07062133312225342,
    #     0.0030260086059570312,
    #     0.0548360288143158
    # ]
    # tmp2 = temp[8:14]
    # temp[8:14] = temp[14:20]
    # temp[14:20] = tmp2
    # temp[9]*=-1
    # temp[13]*=-1
    # temp[15]*=-1
    # temp[19]*=-1
    # tmp2 = temp[2:13]
    # temp[2:13] = temp[14:25]
    # temp[14:25] = tmp2
    # tmp3 = temp[2:25]
    # tmp3 = [i * -1 for i in tmp3]
    # temp[2:25] = tmp3
    # print temp
    # temp2 = [-0.056155614852905274, 0.0230294132232666, 1.5719859266281129, 0.2054372549057007, -0.009169173240661622, -0.0367741584777832, 0.023044753074645995, 0.14054403305053711, 5.730152130126953e-05, 4.1961669921875e-05, -0.40224945545196533, 0.9542288112640381, -0.5492138862609863, 7.264137268066406e-05, 0.0, 4.1961669921875e-05, -0.402486777305603, 0.9511218070983887, -0.551246862411499, 4.1961669921875e-05, 1.5651515769958495, -0.20447806835174562, 0.004559993743896484, 0.04592393398284912, 0.01236799955368042, 0.05284005403518677]
    #
    # print temp2[8:13]
    # print temp2[14:19]
    for i in range(10):
        pose_switcher.switchTo("prepare_left_kick", "walking_pose")
        time.sleep(1)
        pose_switcher.switchTo("prepare_right_kick", "walking_pose")
    # pose_handler.set_pose("walking_pose", 0.5)
    # pose_handler.set_pose("prepare_left_kick", 0.5)
    # pose_handler.set_pose("prepare_left_kick_2", 0.5)
    # pose_handler.set_pose("prepare_left_kick_3", 0.5)
    # pose_handler.set_pose("left_leg_up", 0.3)
    # pose_handler.set_pose("left_kick", 0.1)
    # # pose_handler.set_pose("after_left_kick", 0.3)
    # pose_handler.set_pose("after_left_kick_2", 0.3)
    # pose_handler.set_pose("walking_pose", 0.5)

    # pose_handler.set_pose("left_leg_back", 1.0)
    # pose_handler.set_pose("walking_pose", 1.0)

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


    # stance = [310.0, 14.5, 100.0, 0.05, 0.0, 0.1]
    # step = [0.6, 0.45, 10.0, 0.0, 70.0, -50.0, 70.0, 0.35, 70.0, 70.0, 0.35, 1.0]
    # zmp = [0.0, 0.9, 4.0, 4.0, 0.01, 6.6]
    # hack = [0.05, 0.05]
    # sensor = [0.0,
    #          0.0,
    #          0.0,
    #          0.0,
    #          0.0,
    #          0.0,
    #          0.0,
    #          0.0]
    # stiff = [0.85, 0.3, 0.4, 0.3, 0.2, 0.2]
    # odo = [1.0, 1.0, 1.3]
    # arm = [0.0]
    # robot.locomotion.gait(stance, step, zmp, hack, sensor, stiff, odo, arm)
    #

    # robot.locomotion.autoapply.enable(True)
    # robot.locomotion.odo3etry(True)
    # print robot.locomotion.odometry()
    # robot.locomotion.odometry(True)
    # print robot.locomotion.odometry()
    #
    # #
    # y = 1000.0
    # while True:
    #     robot.kinematics.lookAt(y, 0.0, 100.0, False)
    #     # print robot.kinematics.getHead(True)
    #     # y -= 100.0
    #     y = -y
    #     time.sleep(2)
    # camc = CamCalib(robot)
    # camc.calibrateCamera(True)
    # cam = CamGeom("config/cameras.json", robot)
    # cam.imagePixelToWorld(50, 50, False)
    # walk = Walker(robot)
    # print walk.get_speed()
    # walk.go_to((000.0, 500.0), 100.0)

    # walk.go_around((200.0, 0.0), 20.0)
    # walk.set_speed(0.0, 0.0, 0.0)
    # start = time.time()
    # time.sleep(10.0)
    # print robot.locomotion.odometry()['data']

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
    # robot.joints.hardness(0.0)