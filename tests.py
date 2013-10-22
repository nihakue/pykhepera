import robot
import sys
import numpy as np
import utils
import time

def test_nav():
    r = robot.Robot()
    pose = {'x': 0, 'y':0, 'theta':np.pi/2}
    destination = {'x':0, 'y':0}
    destination_x = raw_input('enter destination x')
    destination['x'] = int(destination_x)
    destination_y = raw_input('enter destination y')
    destination['y'] = int(destination_y)
    distance = utils.wu_to_point(destination, pose)
    angle = utils.rotation_to_point(destination, pose)
    print 'distance between pose and destination: ', distance
    print 'angle between pose and destination: ', angle
    print 'going to destination'

    '''go to point from pose'''
    r.r.rotate(angle)
    time.sleep(2)
    r.r.travel(distance)

