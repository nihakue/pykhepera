

import pykhepera, behaviors
from data import Data
import time
import matplotlib.pyplot as plt
import numpy as np
import pdb
import threading

class Robot(object):
    '''This is the logic/execution module for the pykhepera robot. It handles the
    high level functions such as reloading the robot, controlling the robot,
    etc.
    '''
    def __init__(self):
        self.data = Data()
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1,1,1)
        self.ax.axis([-1000, 1000, -1000, 1000])
        self.line, = self.ax.plot(self.data.x_positions, self.data.y_positions)
        self.r = pykhepera.PyKhepera()
        self.axel_l = 53.0 #self.axis length in mm
        self.running = True

    def update_data(self):
        self.data.sensor_values = self.r.read_sensor_values()
        self.data.wheel_values = self.r.read_wheel_values()

    def to_mm(self, wheel_value):
        return float(wheel_value * 0.08)

    def to_wu(self, mm):
        return int(mm/0.08)

    def get_omega(self):
        vl, vr = self.data.wheel_speeds
        if vr == vl:
            return 0
        omega = ((vr - vl)/self.axel_l)
        return omega

    def get_R(self):
        vl, vr = self.data.wheel_speeds
        if vl == vr:
            return 0
        R = (self.axel_l/2) * ((vl + vr)/(vr - vl))
        return R

    def get_ICC(self):
        x = self.data.x_positions[-1]
        y = self.data.y_positions[-1]
        R = self.get_R()
        theta = self.data.theta

        ICC = ((x - (R * np.sin(theta))), (y + (R * np.cos(theta))))
        return ICC

    def calculate_pose(self, dt):
        x = self.data.x_positions[-1]
        y = self.data.y_positions[-1]
        theta = self.data.theta

        vl, vr = self.data.wheel_speeds
        if vl == vr:
            pose = (np.array([x, y, theta]))
            translation = np.array([vl*np.cos(theta) * dt,
                                   vl*np.sin(theta) * dt, 0])
            pose = pose + translation
        else:
            ICC = self.get_ICC()
            ICCx = ICC[0]
            ICCy = ICC[1]
            omega = self.get_omega()

            rotation_matrix = np.array(
                [
                [np.cos(omega * dt), -np.sin(omega*dt), 0],
                [np.sin(omega * dt), np.cos(omega * dt), 0],
                [0,                  0,                  1]
                ])

            ICC_vector = np.array([x - ICCx, y-ICCy, theta])

            reposition_vector = np.array([ICCx, ICCy, omega * dt])


            pose = np.dot(rotation_matrix, ICC_vector) + reposition_vector

        self.data.x_positions.append(pose[0])
        self.data.y_positions.append(pose[1])
        self.data.theta = pose[2]



    def calibrate(self):
        self.update_data()
        self.r.turn((1,-4))
        while (self.data.wheel_values[0] + self.data.wheel_values[1]) < 2078:
            self.update_data()
        self.r.turn((0,0))


    def calibrate_min(self):
        self.r.reset_wheel_counters()
        self.update_data()
        mins = self.data.sensor_values
        self.r.turn((5,-5))
        while self.data.wheel_values[0] <  2000:
            self.update_data()
            aux = self.data.sensor_values
            for i, val in enumerate(aux):
                if val > mins[i]:
                    mins[i] = val
        self.r.turn((0,0))
        m = 0
        for val in mins:
            m += val
        m = m/8
        self.data.thresholds['mself.ax_ir_reading'] = m
        self.data.thresholds['wall_mself.ax'] = m
        if m:
            print 'calibration succesful: ', mins
            fout = open('data_calibration.data', 'a')
            fout.write('%d:%s' % (m, ''.join(str(mins))))
            fout.write('\n')
            fout.close()


    def update_plot(self):
        self.line.set_xdata(self.data.x_positions)
        self.line.set_ydata(self.data.y_positions)
        self.fig.canvas.draw()

    def will_collide(self):
        for val in self.data.sensor_values[1:5]:
            if val > self.data.thresholds['max_ir_reading']:
                return True

    def state_led(self, number):
        '''converts a number to binary and lights LED accordingly'''
        if number > 3:
            return
        binary = bin(number)[2:]
        if len(binary) == 1:
            binary = '0' + binary
        for i, bit in enumerate(binary):
            self.r.led(i, bit)

    def start(self):
        obj_avoid = behaviors.ObjAvoid(self.data)
        wall_follow = behaviors.WallFollow(self.data)
        go_home = behaviors.GoHome(self.data)
        last_time = time.time()
        start_time = last_time
        time_up = False
        try:
            while self.running:
                current_time = time.time()
                if (not time_up and current_time - start_time > 5):
                    time_up = True
                    if self.r.state is not 1:
                        self.r.state = 3

                dt = (current_time - last_time)
                last_time = time.time()
                self.update_data()
                self.calculate_pose(dt)
                self.update_plot()

                speed = (0,0)
                vals = self.data.sensor_values
                if self.r.state is 0:
                    self.state_led(0)
                    if self.will_collide():
                        speed = (0,0)
                        self.r.state = 1
                    else:
                        speed = (5,5)
                if self.r.state is 1:
                    self.state_led(1)
                    speed = obj_avoid.step()
                    if speed == (5, 5):
                        if time_up:
                            self.r.state = 3
                        else:
                            self.r.state = 2
                            wall_follow.prev_vals = vals
                        speed = (0,0)
                elif self.r.state is 2:
                    self.state_led(2)
                    if self.will_collide():
                        self.r.state = 1
                        continue
                    if (vals[0] < self.data.thresholds['wall_min']
                        and vals[5] < self.data.thresholds['wall_min']):
                        #"No wall :( "
                        self.r.state = 0
                        continue
                    speed = wall_follow.step()
                    #print "Following :)"
                elif self.r.state is 3:
                    print 'going home'
                    self.state_led(3)
                    self.r.stop()
                    rotation = go_home.step()
                    print 'wheel values: ', self.data.wheel_values
                    print 'rotation: ', rotation
                    if rotation[0] != self.data.wheel_values[0]:
                        self.r.set_values('C', rotation)
                        time.sleep(3)
                        self.r.state = 0
                    speed = (5, 5)

                if speed:
                    self.data.wheel_speeds = speed
                    self.r.turn(speed)
        except KeyboardInterrupt:
            print 'killing and cleaning up'
            self.r.purge_buffer()
            self.state_led(0)
            self.r.stop()
            self.r.kill()
