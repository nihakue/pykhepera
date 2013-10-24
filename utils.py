import numpy as np
import pdb

class Point(object):
    """This is a point in the x y plane"""
    def __init__(self, x=0, y=0):
        super(Point, self).__init__()
        self.x = x
        self.y = y

    def __string__(self):
        return "POINT"

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

