'''This is the logic/execution module for the pykhepera robot. It handles the
high level functions such as reloading the robot, controlling the robot,
etc.
'''

import serial
import pykhepera
import behaviors
import time

_sensor_data = {
    'n': [],
    'h': []
}

_thresholds = {
    'max_ir_reading': 120, # This represents the min distance
    'wall_max': 250,
    'wall_min': 75
}

def read_sensor_data():
    for key in _sensor_data:
        _sensor_data[key] = r.get_values(key)

def restart():
    r = load()
    start(r)

def load():
    reload(pykhepera)
    r = pykhepera.PyKhepera()
    return r

def calibrate_min(r):
    mins = r.get_values('n')
    r.set_values('g', [0, -0])
    r.turn(5,-5)
    while r.get_values('h')[0] <  2000:
        aux = r.get_values('n')
        for i, val in enumerate(aux):
            if val > mins[i]:
                mins[i] = val
    print mins
    r.turn(0,0)


def start(r):
    #Test code
    obj_avoid = behaviors.ObjAvoid(_sensor_data, _thresholds)
    wall_avoid = behaviors.WallFollow(_sensor_data, _thresholds)

    while 1:
        read_sensor_data()
        speed = (0,0)
        if r.state is 0:
            speed = obj_avoid.step()
            if speed == (5, 5):
                r.state = 1
                speed = (0,0)
        elif r.state is 1:
            speed =wall_avoid.step()
        turn(speed)

    #TODO: Find a home for this


    #TODO: Find a home for this:
    if (vals[2] > max_ir_reading) or (vals[3] > max_ir_reading):
            r.state = 0
            print "wall in front"
            continue
    #TODO: Find a home for this:
    if vals[0] < self._thresholds['wall_min'] and vals[5] < self.thresholds['wall_min']:
            r.state = 0
            print "too far from wall"
            vl = 5
            vr = 5

    #end test code
    

    # #Allow us to interrupt cleanly with ctrl-C
    # except KeyboardInterrupt:
    #     print 'killing and cleaning up'
    #     r.purge_buffer()
    #     r.led(0,0)
    #     r.led(1,0)
    #     r.stop()
    #     r.kill()

r = load()