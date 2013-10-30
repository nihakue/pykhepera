import numpy as np
import pdb

class Pose(object):
    """Robot's pose in the real world. x and y in mm, theta in radians"""
    def __init__(self, x=0, y=0, theta=0):
        super(Pose, self).__init__()
        self.x = x
        self.y = y
        self.theta = theta

class Point(object):
    """This is a point in the x y plane"""
    def __init__(self, x=0, y=0):
        super(Point, self).__init__()
        self.x = x
        self.y = y

    def __string__(self):
        return "POINT"

home_position = Point(x=503, y=484)
home_pose = Pose(x=503, y=484, theta=np.pi/2)
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
    i = 1
    while threshold[i] > reading:
        i += 1
    rang = float(threshold[i-1]-threshold[i])
    pos = float(reading-threshold[i])
    n = float(i-1)
    return n+(pos/rang)

def estimated_reading(distance, threshold):
    up = threshold[int(distance)]
    down = threshold[int(distance+1)]
    read = (distance-int(distance))*(up-down)
    return down+read
    pass

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


def update_pose(start_pose, wheel_speeds, dt):
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

    new_pose = Pose(pose[0], pose[1], pose[2])
    return new_pose

