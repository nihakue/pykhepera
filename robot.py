
import pykhepera, behaviors
from data import Data
import time
import matplotlib.pyplot as plt
import raycasting
import numpy as np
import pdb
import utils


class Pose(object):
    """Robot pose"""
    def __init__(self, x=0, y=0, theta=0):
        super(Pose, self).__init__()
        self.x = x
        self.y = y
        self.theta = theta


class Robot(object):
    '''This is the logic/execution module for the pykhepera robot. It handles the high level functions such as reloading the robot, controlling the robot, etc.
    '''
    def __init__(self):
        super(Robot, self).__init__()
        self.data = Data()
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.ax.axis([-1000, 1000, -1000, 1000])
        self.line, = self.ax.plot(self.data.x_positions, self.data.y_positions)
        self.r = pykhepera.PyKhepera()
        self.axel_l = 53.0  # self.axis length in mm
        self.data.clear()
        self.pose = Pose(utils.home_position.x, utils.home_position.y,
                         np.pi/2)

    def update_data(self):
        self.data.sensor_values = self.r.read_sensor_values()
        # self.data.wheel_speeds = self.r.read_wheel_speeds()
        self.data.wheel_values = self.r.read_wheel_values()

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

    def update_pose(self, dt):
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
        self.pose.x = pose[0]
        self.pose.y = pose[1]
        self.pose.theta = pose[2]

    def calibrate_min(self):
        self.r.reset_wheel_counters()
        self.update_data()
        mins = self.data.sensor_values
        maxs = mins
        self.r.turn((5, -5))
        print self.data.wheel_values
        while self.data.wheel_values[0] < 2000:
            self.update_data()
            aux = self.data.sensor_values
            for i, val in enumerate(aux):
                if val > mins[i]:
                    mins[i] = val
                else:
                    maxs[i] = val
        self.r.turn((0, 0))
        m = 0
        for val in mins:
            m += val
        m = m/8
        maxs_avg = sum((val in maxs))/len(maxs)
        self.data.thresholds['max_ir_reading'] = m
        self.data.thresholds['wall_max'] = m
        self.data.thresholds['wall_min'] = maxs_avg
        if m:
            print 'calibration succesful: ', mins
            fout = open('data_calibration.data', 'a')
            fout.write('%d:%s' % (m, ''.join(str(mins))))
            fout.write('\n')
            fout.close()

    def calibrate_distances(self):
        readings = []
        self.update_data()
        readings = self.data.sensor_values
        readings_delta = []
        for a in range(10):
            self.r.travel(utils.to_wu(-10))
            time.sleep(1)
            self.update_data()
            readings_delta.append([x - y for x,y in
                                  zip(readings,self.data.sensor_values)])
            readings = self.data.sensor_values
        for r in readings_delta:
            print r[2], r[3]

    def update_plot(self):
        self.line.set_xdata(self.data.x_positions)
        self.line.set_ydata(self.data.y_positions)
        self.fig.canvas.draw()

    def will_collide(self):
        for val in self.data.sensor_values[1:5]:
            if val > self.data.thresholds['max_ir_reading']:
                return True

    def led_state(self, number):
        '''converts a number to binary and lights LED accordingly'''
        if number > 3:
            return
        binary = bin(number)[2:]
        if len(binary) == 1:
            binary = '0' + binary
        for i, bit in enumerate(binary):
            self.r.led(i, bit)

    def update(self, dt):
        self.update_data()
        self.update_pose(dt)
        self.update_plot()
        # self.update_expectations(dt)

    def update_expectations(self, dt):
        self.lidar_range = raycasting.exp_range_for_pose(self.pose,
                                                         plot=True)

    def run(self):
        obj_avoid = behaviors.ObjAvoid(self.data)
        wall_follow = behaviors.WallFollow(self.data)
        go_home = behaviors.GoHome(self.data)
        last_time = time.time()
        time_up = False
        try:
            while True:
                current_time = time.time()
                # if (not time_up and current_time - start_time > 5):
                #     time_up = True
                #     if self.r.state is not 0:
                #         self.r.state = 3

                dt = (current_time - last_time)
                last_time = time.time()
                self.update(dt)
                speed = (0, 0)
                vals = self.data.sensor_values
                if self.r.state is 0:
                    self.led_state(0)
                    if self.will_collide():
                        speed = (0, 0)
                        self.r.state = 1
                    else:
                        speed = (5, 5)
                if self.r.state is 1:
                    self.led_state(1)
                    speed = obj_avoid.step()
                    if speed == (5, 5):
                        if time_up:
                            self.r.state = 3
                        else:
                            self.r.state = 2
                            wall_follow.prev_vals = vals
                        speed = (0, 0)
                elif self.r.state is 2:
                    self.led_state(2)
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
                    self.led_state(3)
                    self.r.stop()
                    suggestion = go_home.step()
                    rotation = suggestion['rotation']
                    print 'wheel values: ', self.data.wheel_values
                    print 'rotation: ', rotation
                    self.r.rotate(rotation)
                    time.sleep(2)
                    self.r.state = 0
                if speed:
                    self.r.turn(speed)
                    self.data.wheel_speeds = speed
        except KeyboardInterrupt:
            print 'killing and cleaning up'
            self.r.purge_buffer()
            self.led_state(0)
            self.r.stop()
            self.r.kill()
