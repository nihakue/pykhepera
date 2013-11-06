from utils import Pose
from math import sin
from math import cos


def calculate_new_pose(pose, wheel_speeds, dt):
    radius = 26
    new_pose = Pose(pose.x, pose.y, pose.theta)
    vl = wheel_speeds[0]
    vr = wheel_speeds[1]
    new_pose.x = pose.x + (0.5 * (vl + vr) * cos(pose.theta))
    new_pose.y = pose.y + (0.5 * (vl + vr) * sin(pose.theta))
    new_pose.theta = pose.theta - (0.5*(vl - vr)/(2*radius))
    return new_pose
