import numpy as np
import pdb
from collections import namedtuple


class Pose(object):
    """Robot's pose in the real world. x and y in mm, theta in radians"""
    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta

    def arr(self):
        return np.array([self.x, self.y, self.theta])


class Particle(Pose):
    """just a pose with a weight"""
    def __init__(self, x=0, y=0, theta=0, w=0.01):
        super(Particle, self).__init__(x, y, theta)
        self.w = w


class Point(object):
    """This is a point in the x y plane"""
    def __init__(self, x=0, y=0):
        super(Point, self).__init__()
        self.x = x
        self.y = y

    def __string__(self):
        return "POINT"

home_position = Point(x=503, y=484)
home_pose = Pose(x=503, y=484, theta=0)
axel_l = 53.0

def to_mm(wheel_value):
        return float(wheel_value * 0.08)

def to_wu(mm):
    return int(mm/0.08)

def rotated_vector(vector, angle):
        return vector.dot(np.array([[np.cos(angle), -np.sin(angle)],
                          [np.sin(angle), np.cos(angle)]]))
def angle_between(v1, v2):
    angle = np.arccos((v1.dot(v2)/np.linalg.norm(v1)) * np.linalg.norm(v2))
    return angle

def codirectional(v1, v2, error=1):
    dot_product = v1.dot(v2)
    magnitudes = np.linalg.norm(v1) * np.linalg.norm(v2)
    return (abs(dot_product - magnitudes) < error)

def rotation_to_point(destination, pose):

    '''takes two dictionaries, returns a rotation in radians
    that, when executed, will face the caller towards the destination'''

    theta = pose.theta
    x = pose.x
    y = pose.y
    destination_vector = vector_to_point(destination, pose)
    theta_vector = np.array([np.cos(theta), np.sin(theta)])
    rotation = angle_between(destination_vector, theta_vector)
    test_vector = rotated_vector(theta_vector, rotation)
    if not codirectional(destination_vector, test_vector):
        print 'trying other rotation'
        rotation = -rotation

    return rotation

def vector_to_point(destination, pose):
    '''given a CURRENT pose'''
    x = destination.x - pose.x
    y = destination.y - pose.y
    return np.array([x, y])

def wu_to_point(destination, pose):
    '''wheel units to point'''
    destination_vector = vector_to_point(destination, pose)
    mm_distance = np.linalg.norm(destination_vector)
    wu_distance = to_wu(mm_distance)
    return wu_distance

def mm_between(destination, origin):
    '''mm between two points'''
    destination_vector = vector_to_point(destination, origin)
    mm_distance = np.linalg.norm(destination_vector)
    return mm_distance


def estimated_distance(reading, threshold):
    upper, lower = reverse_insort(threshold, reading)
    rang = float(threshold[upper] - threshold[lower])
    pos = -float(reading-threshold[lower])
    return (upper + 1 + (pos/rang)) * 10


def estimated_reading(distance, threshold):
    df = float(distance)/10.
    if df >= 9:
        return 30
    di = int(distance)/10
    # read = (distance-int(distance))*(down-up)
    up = threshold[di]
    down = threshold[di + 1]
    read = (df - di) * (up-down)
    return int(up - read)


def reverse_insort(a, x, lo=0, hi=None):
    """return the indexes that would be to the left and right of item x,
    if x were bisected into the list

    If x is already in a, return x as the left

    Optional args lo (default 0) and hi (default len(a)) bound the
    slice of a to be searched.
    """
    if lo < 0:
        raise ValueError('lo must be non-negative')
    if hi is None:
        hi = len(a)
    while lo < hi:
        mid = (lo+hi)//2
        if x > a[mid]:
            hi = mid
        else: lo = mid+1
    if lo >= len(a):
        return len(a) - 2, len(a) - 1
    return lo-1, lo


def get_omega(wheel_speeds):
    vl, vr = wheel_speeds
    if vr == vl:
        return 0
    omega = ((vr - vl)/axel_l)
    return omega


def get_R(wheel_speeds):
    vl, vr = wheel_speeds
    if vl == vr:
        return 0
    R = (axel_l/2) * ((vl + vr)/(vr - vl))
    return R


def get_ICC(pose, wheel_speeds):
    x = pose.x
    y = pose.y
    theta = pose.theta
    R = get_R(wheel_speeds)
    ICC = ((x - (R * np.sin(theta))), (y + (R * np.cos(theta))))
    return ICC


def update_pose(start_pose, wheel_speeds, dt, noisy=False):
    x = start_pose.x
    y = start_pose.y
    theta = start_pose.theta

    vl, vr = wheel_speeds
    if vl == vr:
        pose = (np.array([x, y, theta]))
        translation = np.array([vl*np.cos(theta) * dt,
                               vl*np.sin(theta) * dt, 0])
        pose = pose + translation
    else:
        ICC = get_ICC(start_pose, wheel_speeds)
        ICCx = ICC[0]
        ICCy = ICC[1]
        omega = get_omega(wheel_speeds)

        rotation_matrix = np.array(
            [
            [np.cos(omega * dt), -np.sin(omega*dt), 0],
            [np.sin(omega * dt), np.cos(omega * dt), 0],
            [0,                  0,                  1]
            ])

        ICC_vector = np.array([x - ICCx, y-ICCy, theta])

        reposition_vector = np.array([ICCx, ICCy, omega * dt])

        pose = np.dot(rotation_matrix, ICC_vector) + reposition_vector

    if noisy:
        pose = pose + gauss(2, np.pi/32)
    if type(start_pose) is Particle:
        new_pose = Particle(pose[0], pose[1], pose[2], start_pose.w)
    else:
        new_pose = Pose(pose[0], pose[1], pose[2])
    return new_pose

def gauss(scale, theta_scale):
        xy = np.random.normal(0, scale, 2)
        theta = np.random.normal(0, theta_scale, 1)
        return np.append(xy, theta)

