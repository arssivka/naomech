import cv2
from behavior.robot import Robot
from behavior.camera_geometry import CamGeom
from behavior.camera_calibator import converToCvYUV
import numpy as np

top_camera = False
not_walking_look = True
robot = Robot("192.168.1.64", "5469")
cam = CamGeom("config/cameras.json", robot)

def rotateImage(img, angle):
    rows, cols, dem = img.shape
    M = cv2.getRotationMatrix2D((cols/2,rows/2),angle,1)
    result = cv2.warpAffine(img,M,(cols,rows))
    return result

def onmouse(event, x, y, flags, params):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        pix = cam.imagePixelToWorld(x, y, top_camera)
        robot.kinematics.lookAt(pix[0], pix[1], 0.0, top_camera)
    if event == cv2.EVENT_RBUTTONDBLCLK:
        robot.kinematics.lookAt(100.0, 0.0, 0.0, top_camera)


cv2.namedWindow('mouse_input')
cv2.setMouseCallback('mouse_input', onmouse)

while True:
    raw = robot.cameras.image(top_camera)
    img = converToCvYUV(raw.get('data').data)
    picturerbg = cv2.cvtColor(img, cv2.COLOR_YUV2RGB)
    robot.vision.updateFrame()
    ball = robot.vision.ballDetect()
    pix = cam.imagePixelToWorld(ball["x"] + ball["width"]/2, ball["y"], top_camera)
    if not_walking_look:
        if pix[0] > 0 and ball["width"] > 0:
            print pix
            robot.kinematics.lookAt(pix[0], pix[1], 0.0, top_camera)
        else:
            pass
    cv2.imshow('mouse_input', picturerbg)
    cv2.waitKey(2)