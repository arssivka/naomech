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

    #TODO: test this function
    def odoTranslate(self, x, y, theta):
        tmp_x = (x - self.point.x) * math.cos(self.direction) + (y - self.point.y) * math.sin(self.direction)
        tmp_y = -(x - self.point.x) * math.sin(self.direction) + (y - self.point.y) * math.cos(self.direction)
        self.point.x = tmp_x
        self.point.y = tmp_y

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
    max_distance = 3000

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

    def get_intersect_point(self, rp, point):
        direction2 = math.atan2(point.y, point.x)
        #direction2 = rp.direction
        int_line = g.Line(rp.point,
                          g.Point((rp.point.x + math.cos(direction2) * self.max_distance),
                                  (rp.point.y + math.sin(direction2) * self.max_distance)))
        i_p = g.Point()
        found = False
        dist = self.max_distance
        for l in self.lines:
            temp = l.intersection(int_line)
            if temp != None and temp != float("inf") and self.check_if_point_in_lines(l, int_line, temp):
                tmp_dist = rp.point.distance_to(temp)
                if tmp_dist < dist:
                    i_p = temp
                    dist = tmp_dist
                    found = True
        if found:
            return i_p
        return None



    def check_if_point_in_line(self, l, p):
        return ((p.x >= l.p1.x and p.x <= l.p2.x) or (p.x <= l.p1.x and p.x >= l.p2.x) or (math.fabs(p.x - l.p1.x) <= 0.01)) and \
               ((p.y >= l.p1.y and p.y <= l.p2.y) or (p.y <= l.p1.y and p.y >= l.p2.y) or (math.fabs(p.y - l.p1.y) <= 0.01))

    def check_if_point_in_lines(self, l1, l2, p):
        return self.check_if_point_in_line(l1, p) and self.check_if_point_in_line(l2, p)

    def print_map(self):
        for line in self.lines:
            plt.plot((line.p1.x, line.p2.x), (line.p1.y, line.p2.y), 'g-')
            plt.axis([-4800, 4800, -3600, 3600])

class LocalizationModule:
    map = Map()
    particles_number = 50
    print_once = True

    def __init__(self):
        self.particles = [self.get_random_particle() for i in range(self.particles_number / 2)]
        self.particles.extend([self.get_random_particle(min_x = 0, min_y = 3200, max_x = 4500,
                                                        max_y = 3000, min_dir = math.radians(250),
                                                        max_dir = math.radians(290)) for i in range(self.particles_number / 2)])

    def get_random_particle(self, min_x = 0, min_y = -3200, max_x = 4500, max_y = -3000, min_dir = math.radians(70), max_dir = math.radians(110)):
        return RobotPose(random.uniform(min_x, max_x), random.uniform(min_y, max_y), random.uniform(min_dir, max_dir))

    def sort_particles(self):
        self.particles.sort(key=lambda rp: rp.weight)

    def count_deviations(self):
        arr_x = np.array([rp.point.x for rp in self.particles])
        arr_y = np.array([rp.point.y for rp in self.particles])
        arr_d = np.array([rp.direction for rp in self.particles])
        return (np.std(arr_x), np.std(arr_y), np.std(arr_d))

    def norm_weights(self):
        m = max([rp.weight for rp in self.particles])
        for p in self.particles:
            p.weight /= m

    def resample(self):
        self.particles = [rp for rp in self.particles if rp.weight >= 0.6]
        particles_nedded = self.particles_number - len(self.particles)
        if particles_nedded == self.particles_number:
            self.particles = [self.get_random_particle() for i in range(self.particles_number)]
            return
        if particles_nedded == 0:
            particles_nedded = self.particles/2
            self.particles = self.particles[:particles_nedded]
        xs = (p.point.x for p in self.particles)
        ys = (p.point.y for p in self.particles)
        ds = (p.direction for p in self.particles)
        x = (max(xs), min(xs))
        y = (max(ys), min(ys))
        d = (max(ds), min(ds))
        self.particles = [self.particles, (self.get_random_particle(x[1], y[1], x[0], y[0], d[1], d[0]) for i in range(particles_nedded))]


    def print_plot(self, once = False):
        self.map.print_map()
        for i in range(len(self.particles)):
            self.particles[i].printPose()
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


def main():
     # m = Map()
     # x = 180
     # m.print_map()
     # plt.ion()
     # while True:
     #    x += 10
     #    start = time.time()
     #    m.getIntersectPoint(RobotPose(3700, -1100, math.radians(x)), g.Point(300, 300))
     #    print time.time() - start
     #    m.print_map()
     #    plt.draw()
     #    time.sleep(0.01)
    # prtc = RobotPose(100, 100, 0.5)
    # m.print_map()
    # prtc.printPose()
    # plt.show()
     start = time.time()
     lm = LocalizationModule()
     lm.sort_particles()
     print lm.count_deviations()
     print time.time() - start
     lm.print_plot(True)
    # while True:
    #     lm.printAll()


if __name__ == "__main__":
    main()
