'''This is the logic/execution module for the pykhepera robot. It handles the
high level functions such as reloading the robot, controlling the robot,
etc.
'''

import serial
import pykhepera
import behaviors
import time
import matplotlib.pyplot as plt

_sensor_data = {
    'n': [],
    'h': []
}

_thresholds = {
    'max_ir_reading': 120, # This represents the min distance
    'wall_max': 250,
    'wall_min': 75
}

_trajectory_x = []
_trajectory_y = []
_fig = plt.figure()
_ax = _fig.add_subplot(1,1,1)

def read_sensor_data():
    for key in _sensor_data:
        _sensor_data[key] = r.get_values(key)

def restart():
    r = load()
    start(r)

def load():
    reload(pykhepera)
    _trajectory_x = []
    _trajectory_y = []
    _fig = plt.figure()
    plt.ion()
    _ax = _fig.add_subplot(1,1,1)
    _ax.axis([0, 6, 0, 20])
    r = pykhepera.PyKhepera()
    return r

def calibrate(r):
    read_sensor_data()
    r.set_values('g', [0, -0])
    r.turn((5,5))
    while _sensor_data['h'][0] <  100:
        read_sensor_data()
    r.turn((0,0))


def calibrate_min(r):
    read_sensor_data()
    mins = _sensor_data['n']
    r.set_values('g', [0, -0])
    r.turn((5,-5))
    while _sensor_data['h'][0] <  2000:
        read_sensor_data()
        aux = _sensor_data['n']
        for i, val in enumerate(aux):
            if val > mins[i]:
                mins[i] = val
    #print mins
    r.turn((0,0))
    m = 0
    for val in mins:
        m += val
    m = m/8
    _thresholds['max_ir_reading'] = m

def print_trajectory():
    _trajectory_x.append(_sensor_data['h'][0])
    _trajectory_y.append(_sensor_data['h'][1])
    #print _trajectory_x
    #print _trajectory_y
    _ax.clear()
    _ax.bar(_trajectory_x,_trajectory_y)
    plt.draw()
    plt.show()



def start(r):
    obj_avoid = behaviors.ObjAvoid(_sensor_data, _thresholds)
    wall_follow = behaviors.WallFollow(_sensor_data, _thresholds)

    #print _thresholds['max_ir_reading']

    while 1:
        read_sensor_data()
        print_trajectory()
        speed = (0,0)
        vals = _sensor_data['n']
        if r.state is 0:
            for val in vals[1:5]: #Am I close on the front sensors?
                if val > _thresholds['max_ir_reading']:
                    r.state = 1
                    break
            speed = (5,5)
        if r.state is 1:
            speed = obj_avoid.step()
            if speed == (5, 5):
                r.state = 2
                wall_follow._prev_vals = vals
                speed = (0,0)
        elif r.state is 2:
            for val in vals[2:3]: #Am I close on the front sensors?
                if val > _thresholds['max_ir_reading']:
                    #print "WALL!"
                    r.state = 1
                    continue
            if vals[0] < _thresholds['wall_min'] and vals[5] < _thresholds['wall_min']:
                #print "No wall :( "
                r.state = 0
                continue
            speed = wall_follow.step()
            #print "Following :)"
        r.turn(speed)
    

    # #Allow us to interrupt cleanly with ctrl-C
    # except KeyboardInterrupt:
    #     print 'killing and cleaning up'
    #     r.purge_buffer()
    #     r.led(0,0)
    #     r.led(1,0)
    #     r.stop()
    #     r.kill()

r = load()