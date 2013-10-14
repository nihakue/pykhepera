'''This is the logic/execution module for the pykhepera robot. It handles the
high level functions such as reloading the robot, controlling the robot,
etc.
'''

import serial
import pykhepera, behaviors
from data import Data
import time
import matplotlib.pyplot as plt
import numpy as np

def update_data():
    data.sensor_values = r.read_sensor_values()
    # aux = r.read_wheel_values()
    # data.wheel_values = aux
    # data.wheel_delta = (aux[0]-data.wheel_values[0],
    #     aux[1]-data.wheel_values[1])
    # data.wheel_speeds = r.read_wheel_speeds()

def restart():
    r = load()
    start(r)

def load():
    reload(pykhepera)
    r = pykhepera.PyKhepera()
    r.reset_wheel_counters()
    return r

def to_mm(wheel_value):
    return float(wheel_value * 0.08)

def to_wu(mm):
    return int(mm/0.08)

def get_omega():
    vl, vr = data.wheel_speeds
    if vr == vl:
        return 0
    omega = (vr - vl)/axis_l
    return omega

def get_R():
    vl, vr = data.wheel_speeds
    if vl == vr:
        return 0
    R = (axis_l/2) * ((vl + vr)/(vr - vl))
    return R

def get_ICC():
    x = data.x_positions[-1]
    y = data.y_positions[-1]
    R = get_R()
    theta = data.theta

    ICC = ((x - (R * np.sin(theta))), (y + (R * np.cos(theta))))
    return ICC 

def calculate_pose(dt):
    x = data.x_positions[-1]
    y = data.y_positions[-1]
    theta = data.theta

    vl, vr = data.wheel_speeds
    if vl == vr:
        pose = (np.array([x, y, theta]))
        translation = np.array([vl*np.cos(theta), vl*np.sin(theta), 0])
        pose = pose + translation
        print translation
    else:
        R = get_R()
        ICC = get_ICC()
        ICCx = ICC[0]
        ICCy = ICC[1]
        omega = get_omega()

        rotation_matrix = np.array(
            [
            [np.cos(omega * dt), -np.sin(omega*dt), 0],
            [np.sin(omega * dt), np.cos(omega * dt), 0],
            [0,                  0,                  1]
            ])

        ICC_vector = np.array([x - ICCx, y-ICCy, theta])

        reposition_vector = np.array([ICCx, ICCy, omega * dt])


        pose = np.dot(rotation_matrix, ICC_vector) + reposition_vector

    data.x_positions.append(pose[0])
    data.y_positions.append(pose[1])
    data.theta = pose[2]



def calibrate():
    update_data()
    r.turn((1,-4))
    while (data.wheel_values[0] + data.wheel_values[1]) < 2078:
        update_data()
    r.turn((0,0))


def calibrate_min():
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
    data.thresholds['wall_max'] = m
    if m:
        print 'calibration succesful'

def update_plot():
    line.set_xdata(data.x_positions)
    line.set_ydata(data.y_positions)
    fig.canvas.draw()

def print_trajectory():
    _trajectory_x.append(data.wheel_values[0])
    _trajectory_y.append(data.wheel_values[1])
    #print _trajectory_x
    #print _trajectory_y
    ax.clear()
    ax.bar(_trajectory_x,_trajectory_y)
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

def start():
    r = load()
    obj_avoid = behaviors.ObjAvoid(data)
    wall_follow = behaviors.WallFollow(data)
    last_time = time.time()
    try:
        while 1:
            dt = (time.time() - last_time) * 1000 #convert to ms
            last_time = time.time()
            update_data()
            calculate_pose(dt)
            update_plot()

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
                data.wheel_speeds = speed
                r.turn(speed)
    except KeyboardInterrupt:
        print 'killing and cleaning up'
        r.purge_buffer()
        r.led(0,0)
        r.led(1,0)
        r.stop()
        r.kill()

data = Data()
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(1,1,1)
ax.axis([-1000, 1000, -1000, 1000])
line, = ax.plot(data.x_positions, data.y_positions)

r = load()
axis_l = 53 #axis length in mm