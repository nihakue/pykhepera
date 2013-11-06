import numpy as np
import json
import time
from collections import OrderedDict
from utils import Particle

class Data(object):
    """Data model for sensor values, wheel and position
    information, and odometry"""

    _sensor_values = [0,0,0,0,0,0,0,0]
    _wheel_values = [0, 0]
    _wheel_delta = [0,0]
    _x_positions = []
    _y_positions = []
    _wheel_speeds = [0, 0]
    _theta = np.pi/2
    _pose = Particle()

    _thresholds = OrderedDict([
    ('max_ir_reading', 120), # This represents the min distance
    ('wall_max', 250),
    ('wall_min', 20),
    ('sensor0', [0,0,0,0,0,0,0,0,0,0]),
    ('sensor1', [0,0,0,0,0,0,0,0,0,0]),
    ('sensor2', [0,0,0,0,0,0,0,0,0,0]),
    ('sensor3', [0,0,0,0,0,0,0,0,0,0]),
    ('sensor4', [0,0,0,0,0,0,0,0,0,0]),
    ('sensor5', [0,0,0,0,0,0,0,0,0,0]),
    ('sensor6', [0,0,0,0,0,0,0,0,0,0]),
    ('sensor7', [0,0,0,0,0,0,0,0,0,0])
    ])

    _distance_thresholds = []

    def clear(self):
        self._x_positions = [0]
        self._y_positions = [0]

    @property
    def pose(self):
        return self._pose
    @pose.setter
    def pose(self, value):
        self._pose = value

    @property
    def sensor_values(self):
        return self._sensor_values
    @sensor_values.setter
    def sensor_values(self, value):
        self._sensor_values = value

    @property
    def wheel_values(self):
        return self._wheel_values
    @wheel_values.setter
    def wheel_values(self, value):
        self._wheel_values = value

    @property
    def thresholds(self):
        return self._thresholds
    @thresholds.setter
    def thresholds(self, value):
        self._thresholds = value

    @property
    def distance_thresholds(self):
        if not self._distance_thresholds:
            self._distance_thresholds = [val for key, val
                in self._thresholds.items() if 'sensor' in key]
        return self._distance_thresholds

    @property
    def x_positions(self):
        return self._x_positions
    @x_positions.setter
    def x_positions(self, value):
        self._x_positions = value

    @property
    def y_positions(self):
        return self._y_positions
    @y_positions.setter
    def y_positions(self, value):
        self._y_positions = value

    @property
    def wheel_delta(self):
        return self._wheel_delta
    @wheel_delta.setter
    def wheel_delta(self, value):
        self._wheel_delta = value

    @property
    def wheel_speeds(self):
        return self._wheel_speeds
    @wheel_speeds.setter
    def wheel_speeds(self, value):
        self._wheel_speeds = [val * 8 for val in value] #convert to mm/s

    @property
    def theta(self):
        return self._theta
    @theta.setter
    def theta(self, value):
        self._theta = value

    def save_calibration(self, filename='distance_calibration.data'):
        with open(filename, 'a') as out_file:
            calibration_event = self.thresholds
            calibration_event['day'] = time.localtime().tm_mday
            d_out = json.dumps(calibration_event, sort_keys=True)
            out_file.write(d_out)
            out_file.write('\n')

    def load_calibration(self, filename='distance_calibration.data'):
        try:
            with open(filename) as in_file:
                calibs = [line for line in in_file]
                for line in calibs:
                    indata = json.loads(line)
                    # if indata['day'] == time.localtime().tm_mday:
                    if indata['day'] == time.localtime().tm_mday:
                        self.thresholds = indata
                        break
                else:
                    print 'need new calibration data. please calibrate'
                    self.thresholds = json.loads(calibs[-1])

        except IOError:
            print 'no data_calibration file found. setting to defaults'


