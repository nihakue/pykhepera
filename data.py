import numpy as np

class Data(object):
    """Data model for sensor values, wheel and position
    information, and odometry"""

    _sensor_values = []
    _wheel_values = []
    _wheel_delta = []
    _x_positions = [0]
    _y_positions = [0]
    _wheel_speeds = [0, 0]
    _theta = np.pi/2

    _thresholds = {
    'max_ir_reading': 120, # This represents the min distance
    'wall_max': 250,
    'wall_min': 20
    }

    def clear(self):
        self._x_positions = [0]
        self._y_positions = [0]

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



