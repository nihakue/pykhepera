'''This is the logic/execution module for the pykhepera robot. It handles the
high level functions such as reloading the robot, controlling the robot,
etc.
'''

import serial
import pykhepera, behaviors
from data import Data
import time
import matplotlib.pyplot as plt

def update_data():
    data.sensor_values = r.read_sensor_values()
    data.wheel_values = r.read_wheel_values()
    print 'data sensor:', data.sensor_values

def restart():
    r = load()
    start(r)

def load():
    reload(pykhepera)
    # _trajectory_x = []
    # _trajectory_y = []
    # _fig = plt.figure()
    # plt.ion()
    # _ax = _fig.add_subplot(1,1,1)
    # _ax.axis([0, 6, 0, 20])
    r = pykhepera.PyKhepera()
    return r

def calibrate(r):
    update_data()
    r.set_values('g', [0, -0])
    r.turn((5,5))
    while data.wheel_values[0] <  100:
        update_data()
    r.turn((0,0))

def calibrate_min(r):
    update_data()
    mins = data.sensor_values
    r.set_values('g', [0, -0])
    r.turn((5,-5))
    while data.wheel_values[0] <  2000:
        update_data()
        aux = data.sensor_values
        for i, val in enumerate(aux):
            if val > mins[i]:
                mins[i] = val
    r.turn((0,0))
    m = 0
    for val in mins:
        m += val
    m = m/8
    data.thresholds['max_ir_reading'] = m

def print_trajectory():
    _trajectory_x.append(data.wheel_values[0])
    _trajectory_y.append(data.wheel_values[1])
    #print _trajectory_x
    #print _trajectory_y
    _ax.clear()
    _ax.bar(_trajectory_x,_trajectory_y)
    plt.draw()
    plt.show()

def will_collide():
    for val in data.sensor_values[1:5]:
        if val > data.thresholds['max_ir_reading']:
            return True

def state_LED(number):
    '''converts a number to binary and lights LED accordingly'''
    if number > 3:
        return
    binary = bin(number)[2:]
    if len(binary) == 1:
        binary = '0' + binary
    for i, bit in enumerate(binary):
        r.led(i, bit)

def start(r):
    r = load()
    obj_avoid = behaviors.ObjAvoid(data)
    wall_follow = behaviors.WallFollow(data)
    try:
        while 1:
            update_data()
            # print_trajectory()
            speed = (0,0)
            vals = data.sensor_values
            if r.state is 0:
                state_LED(0)
                if will_collide():
                    speed = (0,0)
                    r.state = 1
                else:
                    speed = (5,5)
            if r.state is 1:
                state_LED(1)
                speed = obj_avoid.step()
                if speed == (5, 5):
                    r.state = 2
                    wall_follow.prev_vals = vals
                    speed = (0,0)
            elif r.state is 2:
                state_LED(2)
                for val in vals[2:3]: #Am I close on the front sensors?
                    if val > data.thresholds['max_ir_reading']:
                        #print "WALL!"
                        r.state = 1
                        continue
                if vals[0] < data.thresholds['wall_min'] and vals[5] < data.thresholds['wall_min']:
                    #print "No wall :( "
                    r.state = 0
                    continue
                speed = wall_follow.step()
                #print "Following :)"
            if speed:
                r.turn(speed)
    except KeyboardInterrupt:
        print 'killing and cleaning up'
        r.purge_buffer()
        r.led(0,0)
        r.led(1,0)
        r.stop()
        r.kill()
    
data = Data()
r = load()