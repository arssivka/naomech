import numpy as np
import cv2

def converToCvYUV(img):
    PIXEL_SIZE_YUV422 = 2
    w = 320
    h = 240
    pixels = np.array([[(
                           ord(img[PIXEL_SIZE_YUV422 * (i * w + j)]),
                           ord(img[PIXEL_SIZE_YUV422 * (i * w + j) + 1 - ((j & 1)<<1)]),
                           ord(img[PIXEL_SIZE_YUV422 * (i * w + j) + 3 - ((j & 1)<<1)]),
                       ) for j in xrange(w)] for i in xrange(h)], dtype = np.uint8)
    return pixels

class CamCalib:
    def __init__(self, robot, square_size = 26):
        self.robot = robot
        self.square_size = square_size


    def calibrateCamera(self, top_camera):
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        objp = np.zeros((6*9,3), np.float32)
        objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
        objp *= self.square_size

        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in imageex plane.

        cv2.namedWindow('Corners', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Camera', cv2.WINDOW_NORMAL)

        count = 0
        while True:
            raw = self.robot.cameras.image(top_camera)
            img = converToCvYUV(raw.get('data').data)
            picturerbg = cv2.cvtColor(img, cv2.COLOR_YUV2RGB)
            gray = cv2.cvtColor(picturerbg,cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, (9,6),None)
            cv2.imshow('Camera', picturerbg)
            cv2.waitKey(2)
            if ret == True:
                #print "Corners!!!!"
                objpoints.append(objp)

                cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                #print corners
                imgpoints.append(corners)
                count += 1
                #Draw and display the corners
                cv2.drawChessboardCorners(picturerbg, (9,6), corners,ret)
                cv2.imshow('Corners', picturerbg)
                cv2.waitKey(500)
                if count == 25:
                    print "25 frames passed to calculate calibration parameters"
                    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
                    print mtx
                    break
            else:
                #print "no corners"
                cv2.imshow('Corners', picturerbg)
                cv2.waitKey(5)


