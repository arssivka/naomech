import numpy as np
import json
import math

class CamGeom:
    def __init__(self, filename, robot, w = 320, h = 240):
        self.loadParams(filename)
        self.focal_x = 0
        self.focal_y = 1
        self.optical_center_x = 2
        self.optical_center_y = 3
        self.open_angle_diag = math.radians(72.6)
        self.focal_length = (0.5 * math.sqrt(math.pow(w, 2) + math.pow(h, 2))) / math.tan(0.5 * self.open_angle_diag)
        self.opt_c_x = w/2
        self.opt_c_y = h/2
        self.robot = robot

    def loadParams(self, filename):
        with open(filename) as config:
            p = json.load(config)
            for keys in p:
                if keys == "top":
                    self.top_parameters = p[keys]
                else:
                    self.bot_parameters = p[keys]


    def imagePixelToWorld(self, img_x, img_y, top_camera, object_height = 0.0):
        tr_matrix = self.robot.kinematics.getHead(top_camera).get('data')
        print tr_matrix
        translation = np.array(tr_matrix[0:3])
        rotation = np.array([[tr_matrix[3]],
                             [tr_matrix[4]],
                             [tr_matrix[5]]])
        r_x = np.array([[1.0, 0.0, 0.0],
                        [0.0, math.cos(rotation[0][0]), -math.sin(rotation[0][0])],
                        [0.0, math.sin(rotation[0][0]), math.cos(rotation[0][0])]])

        r_y = np.array([[math.cos(rotation[1][0]), 0.0, math.sin(rotation[1][0])],
                        [0.0, 1.0, 0.0],
                        [-math.sin(rotation[1][0]), 0.0, math.cos(rotation[1][0])]])

        r_z = np.array([[math.cos(rotation[2][0]), -math.sin(rotation[2][0]), 0.0],
                        [math.sin(rotation[2][0]), math.cos(rotation[2][0]), 0.0],
                        [0.0, 0.0, 1.0]])
        r = r_z.dot(r_y)
        r = r.dot(r_x)
        ###TO CAMERA COORDINATES
        if top_camera:
            pixel = np.array([self.top_parameters[self.focal_x], -img_x + self.top_parameters[self.optical_center_x],
                              -img_y + self.top_parameters[self.optical_center_y]])
        else:
            pixel = np.array([self.bot_parameters[self.focal_x], -img_x + self.bot_parameters[self.optical_center_x],
                              -img_y + self.bot_parameters[self.optical_center_y]])

        pixel = r.dot(pixel)
        ###END TO CAMERA COORDINATES

        heightDiff = object_height - translation[2]
        print heightDiff/pixel[2]
        pixel *= (heightDiff/pixel[2])
        print heightDiff
        pixel += translation
        return [float(pixel[0]), float(pixel[1]), object_height] #normally there will be always true z coordinate, but just in case object height is passed





