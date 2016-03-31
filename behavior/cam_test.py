import cv2
from robot import Robot
from camera_geometry import CamGeom
import camera_calibator
import time
import numpy as np

top_camera = False
not_walking_look = False
robot = Robot("192.168.0.14", "5469")
cam = CamGeom("config/cameras.json", robot)

def rotateImage(img, angle):
    rows, cols, dem = img.shape
    M = cv2.getRotationMatrix2D((cols/2,rows/2),angle,1)
    result = cv2.warpAffine(img,M,(cols,rows))
    return result

def onmouse(event, x, y, flags, params):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        start = time.time()
        pix = cam.imagePixelToWorld(x, y, top_camera)
        print "time", time.time() - start
        print pix
        if not_walking_look:
            robot.kinematics.lookAt(pix[0], pix[1], pix[2], top_camera)
        else:
            joints = robot.kinematics.jointsLookAt(pix[0], pix[1], pix[2], top_camera)
            robot.locomotion.head.positions(joints[0], joints[1])
            robot.locomotion.head.hardness(0.8, 0.8)
    if event == cv2.EVENT_RBUTTONDBLCLK:
        robot.kinematics.lookAt(100.0, 0.0, 0.0, top_camera)


cv2.namedWindow('mouse_input')
cv2.setMouseCallback('mouse_input', onmouse)

while True:
    raw = robot.cameras.image(top_camera)
    img = camera_calibator.converToCvYUV(raw.get('data').data)
    picturerbg = cv2.cvtColor(img, cv2.COLOR_YUV2RGB)
    if top_camera:
        picturerbg = rotateImage(picturerbg, 180.0)
    cv2.imshow('mouse_input', picturerbg)
    cv2.waitKey(2)

