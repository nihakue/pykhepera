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
        self.found_food = False
        self.carrying = 0
        self.data = Data()
        self.plotting = plotting
        self.r = pykhepera.PyKhepera()
        self.axel_l = 53.0  # self.axis length in mm
        self.data.clear()
        self.data.pose = utils.Particle(utils.home_pose.x,
                                        utils.home_pose.y,
                                        utils.home_pose.theta, 0.5)
        self.ppose = utils.home_pose
        self.particle_filter = ParticleFilter(num_particles,
                                              self.data.pose, self.data)
        self.data.load_calibration()
        self.speed_command = (0,0)
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
        self.ppose_lines, = self.ax.plot(self.ppose.x, self.ppose.y, 'go')
        self.im = np.flipud(misc.imread('arena.bmp'))
        plt.imshow(self.im, origin='lower')
        self.plotting_thread = threading.Thread(target=self.update_plot)
        self.plotting_thread.daemon = True
        self.plotting_thread.start()

    def update_data(self):
        self.data.sensor_values = self.r.read_sensor_values()
        wheel_speeds = self.r.read_wheel_speeds()
        self.data.wheel_speeds = self.speed_command
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

    def largest_reading(self, sensor_i, duration=1, sample_rate=.5):
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

    def calibrate_distances(self, closest=2, furthest=4):
        readings = []
        for i in range(9):
            aux = []
            readings.append(aux)
        self.update_data()
        par = -1
        for a in range(9):
            self.r.set_values('G', [0, 0])
            readings[6].append(self.largest_reading(6))
            readings[7].append(self.largest_reading(7))
            self.r.rotate(par * np.pi/2)
            time.sleep(1)
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
            self.r.set_values('C', [0, 0])
            time.sleep(3)
            self.r.travel(utils.to_wu(10))
            time.sleep(1)
        for i in range(8):
            sensor = 'sensor'+str(i)
            self.data.thresholds[sensor] = readings[i]
        # print self.data.thresholds
        # avg_max = sum(r[closest] for r in self.data.distance_thresholds)/len(self.data.distance_thresholds)
        # avg_min = sum(r[furthest] for r in self.data.distance_thresholds)/len(self.data.distance_thresholds)
        # self.data.thresholds['max_ir_reading'] = avg_max
        # self.data.thresholds['wall_min'] = avg_min
        self.data.save_calibration()

    def update_plot(self):
        while self.plotting:
            self.line.set_xdata(self.data.x_positions)
            self.line.set_ydata(self.data.y_positions)
            self.p_lines.set_xdata(self.particle_filter.get_x())
            self.p_lines.set_ydata(self.particle_filter.get_y())
            self.ppose_lines.set_xdata(self.ppose.x)
            self.ppose_lines.set_ydata(self.ppose.y)
            time.sleep(.1)

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
        self.data.pose = utils.update_pose(self.data.pose, self.data.wheel_speeds, dt)
        self.data.x_positions.append(self.data.pose.x)
        self.data.y_positions.append(self.data.pose.y)
        self.data.theta = self.data.pose.theta
        self.particle_filter.update(dt)
        self.ppose = self.particle_filter.most_likely()
        if self.plotting:
            self.fig.canvas.draw()

    def check_food(self):
        if len(self.data.sensor_values) == 0:
            return
        for val in self.data.sensor_values:
            if val < 300:
                return
        else:
            self.found_food = True
            self.carrying += 1

    def flash_leds(self):
        for i in range(4):
            self.led_state(3)
            self.led_state(0)

    def run(self):
        obj_avoid = behaviors.ObjAvoid(self.data)
        # wall_follow = behaviors.WallFollow(self.data)
        # go_home = behaviors.GoHome(self.data)
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
                self.check_food()
                if self.found_food:
                    print "fooooooood!!"
                    self.found_food = False
                    self.r.stop()
                    self.flash_leds()
                self.update(dt)
                speed = (0, 0)
                vals = self.data.sensor_values
                if self.r.state is 0:
                    self.led_state(0)
                    if self.will_collide():
                        speed = (0, 0)
                        self.r.state = 1
                    else:
                        if self.carrying:
                            speed = (5, 5)
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
                    self.speed_command = speed
        except KeyboardInterrupt:
            print 'killing and cleaning up'
            self.r.purge_buffer()
            self.led_state(0)
            self.plotting = False
            self.plotting_thread.join()
            self.r.stop()
            self.r.kill()
            self.particle_filter.tear_down()
