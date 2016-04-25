import numpy as np
import geo2d.geometry as g
import matplotlib.pyplot as plt
import math
import random
import time

class RobotPose:
    point = g.Point(0, 0)
    direction = 0.0
    distance = 100
    weight = 0.0
    def __init__(self, x, y, direct):
        self.point = g.Point(x, y)
        self.direction = direct

    def odoTranslate(self, x, y, theta):
        y *= -1
        dist = math.hypot(x, y)
        angle = math.atan2(y, x)
        self.point.translate(math.cos(self.direction - angle) * dist, math.sin(self.direction - angle) * dist)
        self.direction += theta
        self.direction % (math.pi * 2)


    def printPose(self):
        circle1 = plt.Circle((self.point.x, self.point.y), 100, color = 'r')
        plt.gcf().gca().add_artist(circle1)
        plt.plot((self.point.x, (self.point.x + math.cos(self.direction) * self.distance)),
                 (self.point.y, (self.point.y + math.sin(self.direction) * self.distance)))

class Map:
    central_line_corner_x = 0
    central_line_corner_y = 3000
    corner_x = 4500
    corner_y = 3000
    central_y = 0
    penalty_corners_y = 1100
    penalty_corner_x = 3900
    max_distance = 8000
    enemy_point = g.Point(-4100.0, 0.0)
    friendly_point = g.Point(4100.0, 0.0)

    def __init__(self):

        self.lines = [g.Line(*(line)) for line in [(g.Point(self.central_line_corner_x, self.central_line_corner_y), g.Point(self.central_line_corner_x, -self.central_line_corner_y)),
                                                   (g.Point(self.corner_x, self.central_line_corner_y), g.Point(-self.corner_x, self.central_line_corner_y)),
                                                   (g.Point(self.corner_x, -self.central_line_corner_y), g.Point(-self.corner_x, -self.central_line_corner_y)),
                                                   (g.Point(self.corner_x, self.central_line_corner_y), g.Point(self.corner_x, -self.central_line_corner_y)),
                                                   (g.Point(-self.corner_x, self.central_line_corner_y), g.Point(-self.corner_x, -self.central_line_corner_y)),
                                                   (g.Point(self.penalty_corner_x, self.penalty_corners_y), g.Point(self.penalty_corner_x, -self.penalty_corners_y)),
                                                   (g.Point(-self.penalty_corner_x, self.penalty_corners_y), g.Point(-self.penalty_corner_x, -self.penalty_corners_y)),
                                                   (g.Point(self.corner_x, self.penalty_corners_y), g.Point(self.penalty_corner_x, self.penalty_corners_y)),
                                                   (g.Point(self.corner_x, -self.penalty_corners_y), g.Point(self.penalty_corner_x, -self.penalty_corners_y)),
                                                   (g.Point(-self.corner_x, self.penalty_corners_y), g.Point(-self.penalty_corner_x, self.penalty_corners_y)),
                                                   (g.Point(-self.corner_x, -self.penalty_corners_y), g.Point(-self.penalty_corner_x, -self.penalty_corners_y))]]

    def get_intersect_point(self, rp, point, distance = None):
        # point = g.Point(point.x, -point.y)
        direction2 = math.atan2(point.y, point.x)
        int_line = g.Line(rp.point,
                          g.Point((rp.point.x + math.cos(direction2 + rp.direction) * self.max_distance),
                                  (rp.point.y + math.sin(direction2 + rp.direction) * self.max_distance)))
        i_p = g.Point()
        found = False
        dist = self.max_distance
        neededline = None
        for l in self.lines:
            temp = l.intersection(int_line)
            if temp != None and temp != float("inf") and self.check_if_point_in_lines(l, int_line, temp):
                tmp_dist = rp.point.distance_to(temp)
                if distance is None:
                    if tmp_dist < dist:
                        i_p = temp
                        dist = tmp_dist
                        neededline = l
                        found = True
                else:
                    t_d = abs(distance - tmp_dist)
                    if t_d <= dist:
                        dist = t_d
                        i_p = temp
                        neededline = l
                        found = True
        if found:
            return i_p, neededline
        return None, None


    def check_if_point_in_line(self, l, p):
        return ((p.x >= l.p1.x and p.x <= l.p2.x) or (p.x <= l.p1.x and p.x >= l.p2.x) or (math.fabs(p.x - l.p1.x) <= 0.01)) and \
               ((p.y >= l.p1.y and p.y <= l.p2.y) or (p.y <= l.p1.y and p.y >= l.p2.y) or (math.fabs(p.y - l.p1.y) <= 0.01))

    def check_if_point_in_lines(self, l1, l2, p):
        return self.check_if_point_in_line(l1, p) and self.check_if_point_in_line(l2, p)

    def lines_eq(self, l1, l2):
        return l1.p1.x == l2.p1.x and l1.p1.y == l2.p1.y and l1.p2.x == l2.p2.x and l1.p2.y == l2.p2.y

    def print_map(self):
        for line in self.lines:
            plt.plot((line.p1.x, line.p2.x), (line.p1.y, line.p2.y), 'g-')
            plt.axis([-4800, 4800, -3600, 3600])

class LocalizationModule:
    map = Map()
    particles_number = 100
    print_once = True
    position = RobotPose(0.0, 0.0, 0.0)
    parsed_lines = []
    distances = []

    def __init__(self, robot, cam_geom):
        self.particles = [self.get_random_particle() for i in range(self.particles_number / 2)]
        self.particles.extend([self.get_random_particle(min_x = 0, min_y = 3200, max_x = 4500,
                                                        max_y = 3000, min_dir = math.radians(250),
                                                        max_dir = math.radians(290)) for i in range(self.particles_number / 2)])
        self.robot = robot
        self.cam_geom = cam_geom

    def get_random_particle(self, min_x = 0, min_y = -3200, max_x = 4500, max_y = -3000, min_dir = math.radians(70), max_dir = math.radians(110)):
        return RobotPose(random.uniform(min_x, max_x), random.uniform(min_y, max_y), random.uniform(min_dir, max_dir))

    def sort_particles(self):
        self.particles.sort(key=lambda rp: rp.weight)

    def count_deviations(self):
        arr_x = np.array([rp.point.x for rp in self.particles])
        arr_y = np.array([rp.point.y for rp in self.particles])
        arr_d = np.array([rp.direction for rp in self.particles])
        return (np.std(arr_x), np.std(arr_y), np.std(arr_d))

    def count_mean(self):
        arr_x = np.array([rp.point.x for rp in self.particles])
        arr_y = np.array([rp.point.y for rp in self.particles])
        arr_d = np.array([rp.direction for rp in self.particles])
        return (np.mean(arr_x), np.mean(arr_y), np.mean(arr_d))

    def norm_weights(self):
        m = max([rp.weight for rp in self.particles])
        if m != 0:
            for p in self.particles:
                p.weight /= m

    def resample(self):
        self.particles = [rp for rp in self.particles if rp.weight >= 0.98]
        self.sort_particles()
        particles_nedded = self.particles_number - len(self.particles)
        if particles_nedded == self.particles_number:
            self.particles = [self.get_random_particle() for i in range(self.particles_number/2)]
            self.particles.extend([self.get_random_particle(min_x = 0, min_y = 3200, max_x = 4500,
                                                            max_y = 3000, min_dir = math.radians(250),
                                                            max_dir = math.radians(290)) for i in range(self.particles_number / 2)])
            return
        if particles_nedded == 0:
            particles_nedded = self.particles_number / 2
            self.particles = self.particles[:particles_nedded]
        dev = self.count_deviations()
        x = (max(self.particles, key=lambda rp: rp.point.x).point.x + dev[0] * 0.15, min(self.particles, key=lambda rp: rp.point.x).point.x - dev[0] * 0.15)
        y = (max(self.particles, key=lambda rp: rp.point.y).point.y + dev[1] * 0.15, min(self.particles, key=lambda rp: rp.point.y).point.y - dev[1] * 0.15)
        d = (max(self.particles, key=lambda rp: rp.direction).direction + dev[2] * 0.15, min(self.particles, key=lambda rp: rp.direction).direction - dev[2] * 0.15)
        self.particles.extend([self.get_random_particle(x[1], y[1], x[0], y[0], d[1], d[0]) for i in range(particles_nedded)])

    #TODO: Test this shit first!
    def get_sensors(self):
        self.robot.vision.updateFrame()
        vision_lines = self.robot.vision.lineDetect()
        if len(vision_lines) != 0:
            self.parsed_lines = []
            self.distances = []
            for i in vision_lines:
                c1 = self.cam_geom.imagePixelToWorld(i["x1"], i["y1"], False)
                c2 = self.cam_geom.imagePixelToWorld(i["x2"], i["y2"], False)
                if c1[0] > self.map.max_distance or c1[0] < 0 or c2[0] > self.map.max_distance or c2[0] < 0:
                    continue
                self.parsed_lines.append((c1, c2))
                self.distances.append((math.hypot(c1[0], c1[1]), math.hypot(c2[0], c2[1])))
        # self.print_plot(once=True)



    #TODO: Test this shit second!
    def update_sensors(self, need_get):
        if need_get:
            self.get_sensors()
        start = time.time()
        print "len ", len(self.parsed_lines)
        if len(self.parsed_lines) > 0:
            for p in self.particles:
                for i in range(len(self.parsed_lines)):
                    point1, l1 = self.map.get_intersect_point(p, g.Point(self.parsed_lines[i][0][0], self.parsed_lines[i][0][1]), distance=self.distances[i][0])
                    point2, l2 = self.map.get_intersect_point(p, g.Point(self.parsed_lines[i][1][0], self.parsed_lines[i][1][1]), distance=self.distances[i][1])

                    if point1 is None or point2 is None:# or  not self.map.lines_eq(l1 ,l2):
                        p.weight = 0.0
                        continue
                    else:
                        # if len(self.parsed_lines) > 2:
                        if (math.hypot(point1.x, point1.y) < math.hypot(point2.x, point2.y)):
                            dist = p.point.distance_to(point1)
                            ind = 0
                        else:
                            dist = p.point.distance_to(point2)
                            ind = 1
                        w = abs(dist - self.distances[i][ind])
                        p.weight += (1 - w / self.map.max_distance) / 2
                        # else:
                        # dist = p.point.distance_to(point1)
                        # w = abs(dist - self.distances[i][0])
                        # p.weight += (1 - w / self.map.max_distance) / 2
                        # dist = p.point.distance_to(point2)
                        # w = abs(dist - self.distances[i][1])
                        # p.weight += (1 - w / self.map.max_distance) / 2
        print "update time: ", time.time() - start

    def generate_after_fall_particles(self):
        self.particles = [self.get_random_particle(min_x = self.position.point.x - 200.0, min_y = self.position.point.y - 200,
                                                   max_x = self.position.point.x + 200.0,
                                                        max_y = self.position.point.y + 200, min_dir = self.position.direction - math.radians(10),
                                                        max_dir = self.position.direction + math.radians(10)) for i in range(self.particles_number)]


    #TODO: Test this shit third!
    def localization(self, after_fall = False):
        self.robot.kinematics.lookAt(1000.0, 0.0, 0.0, False)
        look_at_points = [(1000.0, 500.0, 0.0), (1000.0, 0.0, 0.0)]
        index = 0
        sign = -1
        if after_fall:
            self.generate_after_fall_particles()
        count = 0
        update = True
        while self.count_deviations() > (150.0, 150.0, math.radians(10)):
            self.update_sensors(update)
            update = False
            self.norm_weights()
            self.resample()
            count += 1
            if count == 50:
                count = 0
                update = True
                if not after_fall:
                    self.robot.kinematics.lookAt(look_at_points[index][0], math.copysign(look_at_points[index][1], self.count_mean()[1]), look_at_points[index][2], False)
                else:
                    self.robot.kinematics.lookAt(look_at_points[index][0], look_at_points[index][1] * sign, look_at_points[index][2], False)
                    sign *= -1
                time.sleep(0.5)
                index += 1
                if index > 1:
                    index = 0
                # self.print_plot(once=True)
            print "deviations", self.count_deviations()
            print "mean", self.count_mean()
        mean = self.count_mean()
        self.position.point = g.Point(mean[0], mean[1])
        self.position.direction = mean[2]# + math.pi



    def print_plot(self, once = False):
        self.map.print_map()
        # for i in range(len(self.particles)):
        #     # self.particles[i].odoTranslate(1000.0, -1000.0, math.radians(-60))
        #     if self.particles[i].weight > 0.8:
        #         self.particles[i].printPose()
        self.position.printPose()
        if(self.print_once):
            if not once:
                plt.ion()
                self.print_once = False
            plt.show()
        else:
            plt.draw()
            time.sleep(0.05)
            plt.clf()

    def print_console(self):
        for i in self.particles:
            print ('x: ', i.point.x, 'y: ', i.point.y, 'weight: ', i.weight)

    def udapte_odo_pos(self, odometry):
        self.position.odoTranslate(odometry[0], odometry[1], odometry[2])

    def global_to_local(self, x, y):
        new_x = x - self.position.point.x
        new_y = y - self.position.point.y
        rotated_x = new_x * math.cos(self.position.direction) + new_y * math.sin(self.position.direction)
        rotated_y = -new_x * math.sin(self.position.direction) + new_y * math.cos(self.position.direction)
        return (rotated_x, rotated_y, math.atan2(rotated_y, rotated_x))

#This is only for testing puprosesK
class LocaTesting:
    map = Map()
    particles_number = 100
    position = RobotPose(0.0, 0.0, 0.0)
    parsed_lines = []
    def __init__(self, robot, cam_geom):
        self.robot = robot
        self.cam_geom = cam_geom

    def get_sensors(self):
        self.robot.vision.updateFrame()
        vision_lines = self.robot.vision.lineDetect()
        if len(vision_lines) != 0:
            self.parsed_lines = []
            print "vision lines", len(vision_lines)
            for i in vision_lines:
                c1 = self.cam_geom.imagePixelToWorld(i["x1"], i["y1"], False)
                c2 = self.cam_geom.imagePixelToWorld(i["x2"], i["y2"], False)
                if c1[0] > self.map.max_distance or c1[0] < 0 or c2[0] > self.map.max_distance or c2[0] < 0:
                    continue
                else:
                    print "c1 ", c1, "c2 ", c2
                    p1 = g.Point(self.position.point.x, self.position.point.y)
                    p2 = g.Point(self.position.point.x, self.position.point.y)
                    dist = math.hypot(c1[0], c1[1])
                    angle = math.atan2(c1[1], c1[0])
                    p1.translate(math.cos(self.position.direction - angle) * dist, math.sin(self.position.direction - angle) * dist)
                    dist = math.hypot(c2[0], c2[1])
                    angle = math.atan2(c2[1], c2[0])
                    p2.translate(math.cos(self.position.direction - angle) * dist, math.sin(self.position.direction - angle) * dist)
                    print "p1 ", p1, "p2 ", p2
                    self.parsed_lines.append((c1, c2))

    def print_plot(self):
        self.map.print_map()
        self.position.printPose()
        for i in self.parsed_lines:
            plt.plot((i[0][0], i[1][0]),(i[0][1], i[1][1]))
        plt.show()
