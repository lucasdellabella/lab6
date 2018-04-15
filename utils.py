from setting import *
import numpy as np
import random
random.seed(RANDOM_SEED)
import math

""" Some math utilies, feel free to use any of these!!!
    (For particle filtering stuff)
"""

# euclian distance in grid world
def grid_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


# utils for 2d rotation, given frame \theta
def rotate_point(x, y, heading_deg):
    c = math.cos(math.radians(heading_deg))
    s = math.sin(math.radians(heading_deg))
    xr = x * c + y * -s
    yr = x * s + y * c
    return xr, yr


# heading angle difference = heading1 - heading2
# return value always in range (-180, 180] in deg
def diff_heading_deg(heading1, heading2):
	dh = heading1 - heading2
	while dh > 180:
		dh -= 360
	while dh <= -180:
		dh += 360
	return dh


def compute_mean_pose(particles, confident_dist=1):
    """ Compute the mean pose for all particles
    	This is not part of the particle filter algorithm but rather an
    	addition to show the "best belief" for current pose
    """
    m_x, m_y, m_count = 0, 0, 0
    # for rotation average
    m_hx, m_hy = 0, 0
    for p in particles:
        m_count += 1
        m_x += p.x
        m_y += p.y
        m_hx += math.sin(math.radians(p.h))
        m_hy += math.cos(math.radians(p.h))

    if m_count == 0:
        return -1, -1, 0, False

    m_x /= m_count
    m_y /= m_count

    # average rotation
    m_hx /= m_count
    m_hy /= m_count
    m_h = math.degrees(math.atan2(m_hx, m_hy));

    # Now compute how good that mean is -- check how many particles
    # actually are in the immediate vicinity
    m_count = 0
    for p in particles:
        if grid_distance(p.x, p.y, m_x, m_y) < 1:
            m_count += 1

    return m_x, m_y, m_h, m_count > len(particles) * 0.95

# utils to add gaussian noise

def add_gaussian_noise(data, sigma):
    return data + random.gauss(0.0, sigma)

def add_odometry_noise(odom_act, heading_sigma, trans_sigma):
    return (add_gaussian_noise(odom_act[0], trans_sigma), \
        add_gaussian_noise(odom_act[1], trans_sigma), \
        add_gaussian_noise(odom_act[2], heading_sigma))

def add_marker_measurement_noise(marker_measured, trans_sigma, rot_sigma):
    return (add_gaussian_noise(marker_measured[0], trans_sigma), \
        add_gaussian_noise(marker_measured[1], trans_sigma), \
        add_gaussian_noise(marker_measured[2], rot_sigma))


"""
Utils for RRT stuff
"""

class Node(object):
    """Class representing a node in RRT
    """

    def __init__(self, coord, parent=None):
        super(Node, self).__init__()
        self.coord = coord
        self.parent = parent

    @property
    def x(self):
        return self.coord[0]

    @property
    def y(self):
        return self.coord[1]

    def __getitem__(self, key):
        assert (key == 0 or key == 1)
        return self.coord[key]


def get_dist(p, q):
    return np.sqrt((p.x - q.x) ** 2 + (p.y - q.y) ** 2)


################################################################################
######## helper function for line segment intersection check   #################
# credit:http://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect
################################################################################
def is_zero(val):
    return abs(val) < 1e-9


def is_on_segment(p, q, r):
    if (q.x <= max(p.x, r.x) and q.x >= min(p.x, r.x) and
                q.y <= max(p.y, r.y) and q.y >= min(p.y, r.y)):
        return True
    return False


def get_orientation(p, q, r):
    val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)
    if is_zero(val):
        # colinear
        return 0
    elif val > 0:
        # clockwise
        return 1
    else:
        # counter-clockwise
        return 2


def is_intersect(p1, q1, p2, q2):
    o1 = get_orientation(p1, q1, p2)
    o2 = get_orientation(p1, q1, q2)
    o3 = get_orientation(p2, q2, p1)
    o4 = get_orientation(p2, q2, q1)

    if (o1 != o2 and o3 != o4):
        return True
    if (is_zero(o1) and is_on_segment(p1, p2, q1)):
        return True
    if (is_zero(o2) and is_on_segment(p1, q2, q1)):
        return True
    if (is_zero(o3) and is_on_segment(p2, p1, q2)):
        return True
    if (is_zero(o4) and is_on_segment(p2, q1, q2)):
        return True
    return False
