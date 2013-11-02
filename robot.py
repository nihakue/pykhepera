import pykhepera, behaviors
from data import Data
import time
import threading
import matplotlib.pyplot as plt
from scipy import misc
import raycasting
import numpy as np
import utils
from particle_filter import ParticleFilter


class Robot(object):
    '''This is the logic/execution module for the pykhepera robot. It handles the high level functions such as reloading the robot, controlling the robot, etc.
    '''
    def __init__(self, num_particles=100, plotting=False):
        super(Robot, self).__init__()
        self.data = Data()
        self.plotting = plotting
        self.r = pykhepera.PyKhepera()
        self.axel_l = 53.0  # self.axis length in mm
        self.data.clear()
        self.pose = utils.home_pose
        self.particle_filter = ParticleFilter(num_particles,
                                              self.pose, self.data)
        self.data.load_calibration()
        if self.plotting:
            self.init_plotting()

    def init_plotting(self):
        plt.ion()
        self.fig = plt.figure(figsize=(16, 12))
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.ax.axis([0,1469, 0, 962])
        self.line, = self.ax.plot(self.data.x_positions, self.data.y_positions)
        self.p_lines, = self.ax.plot(self.particle_filter.get_x(),
                                     self.particle_filter.get_y(), 'r.')
        self.im = np.flipud(misc.imread('arena.bmp'))
        plt.imshow(self.im, origin='lower')
        self.plotting_thread = threading.Thread(target=self.update_plot)
        self.plotting_thread.daemon = True
        self.plotting_thread.start()

    def update_data(self):
        self.data.sensor_values = self.r.read_sensor_values()
        # self.data.wheel_speeds = self.r.read_wheel_speeds()
        self.data.wheel_values = self.r.read_wheel_values()

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

    def largest_reading(self, sensor_i, duration=2, sample_rate=.5):
        self.update_data()
        max_reading = self.data.sensor_values[sensor_i]
        for i in range(int(duration/sample_rate)):
            self.update_data()
            if self.data.sensor_values[sensor_i] > max_reading:
                max_reading = self.data.sensor_values[sensor_i]
            print 'reading: ', self.data.sensor_values[sensor_i]
            time.sleep(sample_rate)
        print 'max_reading for %d: %d' % (sensor_i, max_reading)
        return max_reading


    def calibrate_distances(self):
        readings = []
        for i in range(8):
            aux = []
            readings.append(aux)
        self.update_data()
        par = -1
        for a in range(10):
            readings[6].append(self.largest_reading(6))
            readings[7].append(self.largest_reading(7))
            self.r.rotate(par * np.pi/2)
            time.sleep(0.3)
            readings[0].append(self.largest_reading(0))
            self.r.rotate(par * np.pi/4)
            time.sleep(0.3)
            readings[1].append(self.largest_reading(1))
            self.r.rotate(par * np.pi/4)
            time.sleep(0.3)
            readings[2].append(self.largest_reading(2))
            readings[3].append(self.largest_reading(3))
            self.r.rotate(par * np.pi/4)
            time.sleep(0.3)
            readings[4].append(self.largest_reading(4))
            self.r.rotate(par * np.pi/4)
            time.sleep(0.3)
            readings[5].append(self.largest_reading(5))
            self.r.rotate(par * np.pi/2)
            time.sleep(0.3)
            self.r.travel(utils.to_wu(10))
            time.sleep(1)
        for i in range(8):
            sensor = 'sensor'+str(i)
            self.data.thresholds[sensor] = readings[i]
        print self.data.thresholds
        self.data.save_calibration()

    def update_plot(self):
        while self.plotting:
            self.line.set_xdata(self.data.x_positions)
            self.line.set_ydata(self.data.y_positions)
            self.p_lines.set_xdata(self.particle_filter.get_x())
            self.p_lines.set_ydata(self.particle_filter.get_y())

            time.sleep(.5)

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
        self.pose = utils.update_pose(self.pose, self.data.wheel_speeds, dt)
        self.data.x_positions.append(self.pose.x)
        self.data.y_positions.append(self.pose.y)
        self.data.theta = self.pose.theta
        self.particle_filter.update(dt)
        if self.plotting:
            self.fig.canvas.draw()

    def food_found(self):
        # m = 0
        # for val in self.data.sensor_values:
        #     m += val
        # m = m/8
        # print "media: ",str(m)
        # if m < 100:
        #     return False
        if len(self.data.sensor_values) == 0:
            return False
        for val in self.data.sensor_values:
            #if (((val-m) > 70) or ((val-m) < -70)):
            if val < 300:
                return False
        return True


    def run(self):
        obj_avoid = behaviors.ObjAvoid(self.data)
        wall_follow = behaviors.WallFollow(self.data)
        go_home = behaviors.GoHome(self.data)
        last_time = time.time()
        time_up = False
        self.update_data()
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
                if self.food_found():
                    print "fooooooood!!"
                    speed = (0, 0)
                    self.r.turn(speed)
                    time.sleep(10)
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
                    # if speed == (5, 5):
                    #     if time_up:
                    #         self.r.state = 3
                    #     else:
                    #         self.r.state = 2
                    #         wall_follow.prev_vals = vals
                    #     speed = (0, 0)
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
