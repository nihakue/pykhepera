from robot import Robot
from robot import Pose
import sys
import numpy as np
import utils
from utils import Point
import time

LOOP = [(560, 293), (916, 293),(1021, 476),
(1128,892), (883, 807), (765, 714),
(535, 703), (503, 484)]

def nav():
    r = Robot()
    pose = Pose(x=0, y=0, theta=np.pi/2)
    destination = Point(x=0, y=0)
    destination_x = raw_input('enter destination x')
    destination.x = int(destination_x)
    destination_y = raw_input('enter destination y')
    destination.y = int(destination_y)
    distance = utils.wu_to_point(destination, pose)
    angle = utils.rotation_to_point(destination, pose)

    '''go to point from pose'''
    r.r.rotate(angle)
    time.sleep(2)
    r.r.travel(distance)

def lap():
    loop = [Point(x=x1,y=y1) for x1,y1 in LOOP]
    r = Robot()
    pose = Pose(x=503, y=484, theta=np.pi/2)
    for point in loop:
        print point.x, point.y
        print 'pose: %d, %d, %.2f' % (pose.x, pose.y, pose.theta)
        distance = utils.wu_to_point(point, pose)
        angle = utils.rotation_to_point(point, pose)
        r.r.rotate(angle)
        time.sleep(3)
        r.r.travel(distance)
        time.sleep(3)
        pose = Pose(x=point.x, y=point.y, theta=pose.theta-angle)


