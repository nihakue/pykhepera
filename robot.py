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
    obj_avoid = behaviors.ObjAvoid(_sensor_data)
    wall_avoid = behaviors.WallFollow(_sensor_data)

    while 1:
        read_sensor_data()
        obj_avoid.step()
        wall_avoid.step()
        time.sleep(1)

    #end test code
    try:
        max_ir_reading = 120 # This represents the min distance
        wall_max = 250
        wall_min = 75
        prev_l = 5
        prev_r = 5
        r.turn(prev_l,prev_r)
        prev_vals = r.get_values('n')
        r.state = 0

        avg_l = [] # average change in distances for wall following
        avg_r = []

        while(1):
            print 'state: ', r.state,
            vl = 5
            vr = 5
            vals = r.get_values('n')
            print 'IR readings: %s' % vals
            
            if r.state is 0:
                r.led(1, 0)
                r.led(0, 1)

                #Reset averages: this effectively disables the running average
                avg_l = []
                avg_r = []
                close = False

                for val in vals[1:5]: #Am I close on the front sensors?
                    if val > max_ir_reading:
                        close = True

                if close: #Close to an object, create a delta command for left and right
                    if(vals[1] > max_ir_reading/1.5):
                        vr -= 4
                    if(vals[2] > max_ir_reading):
                        vr -= 4
                    if(vals[3] > max_ir_reading):
                        vl -= 4
                    if(vals[4] > max_ir_reading/1.5):
                        vl -= 4
                    if((vl == vr) and vl != 5): #May never enter this block
                        vl = -vl
                #Avoid sending the same command over and over
                if (vl, vr) != (prev_l, prev_r): 
                    if vl == 5 and vr == 5:
                        r.state = 2
                        vl = 0
                        vr = 0
                    #Execute the turn command with the delta fused delta variables
                    r.turn(vl,vr) 

            elif r.state is 1: #Align to the wall
                vl = 0
                vr = 0
                r.turn(0,0)

            elif r.state is 2: #Follow the wall
                r.led(0, 1)
                r.led(1, 1)
                prev_vals = r.get_values('n')

                #calculate the change relative to the wall since last step
                dl = vals[0] - prev_vals[0]
                dr = vals[5] - prev_vals[5]
                dl_avg = 0
                dr_avg = 0

                avg_l.append(dl)
                avg_r.append(dr)

                #Calculate the average of the last three distance changes
                if len(avg_l) == 3:
                    total = 0
                    for val in avg_l:
                        total += val
                    dl_avg = total/3
                    total = 0
                    for val in avg_r:
                        total += val
                    dr_avg = total/3
                    print 'list left: ', avg_l
                    avg_l.pop(0) # shift the lists left to make room
                    avg_r.pop(0)

                else:
                    dl_avg = dl
                    dr_avg = dr

                if (vals[2] > max_ir_reading) or (vals[3] > max_ir_reading):
                    r.state = 0
                    print "wall in front"
                    continue

                '''dl and dr represent the change in 'distance' 
                (in terms of an ir reading) between the robot
                and the wall.
                positive: I've moved further away
                negative: I've moved closer closer
                '''
                if vals[0] > vals[5]: #Wall on the left side
                    if dl_avg > 0: #Moving further away (right)
                        print 'turning left'
                        vl -= 3
                    elif dl_avg < 0:#Moving closer (left)
                        print 'turning right'
                        vr -= 3

                elif vals[5] > vals[0]: #Wall on the ride side
                    if dr_avg > 0:
                        print 'turning right'
                        vr -= 3
                    elif dr_avg < 0:
                        print 'turning left'
                        vl -= 3

                if vals[0] < wall_min and vals[5] < wall_min:
                    r.state = 0
                    print "too far from wall"
                    vl = 5
                    vr = 5

                r.turn(vl, vr)

                prev_l = vl
                prev_r = vr
                prev_vals = vals

    #Allow us to interrupt cleanly with ctrl-C
    except KeyboardInterrupt:
        print 'killing and cleaning up'
        r.purge_buffer()
        r.led(0,0)
        r.led(1,0)
        r.stop()
        r.kill()

r = load()