
class Data(object):
    """Data model for sensor values, wheel and position 
    information, and odometry"""

    _sensor_values = []
    _wheel_values = []

    _thresholds = {
    'max_ir_reading': 120, # This represents the min distance
    'wall_max': 250,
    'wall_min': 75
    }

    def __init__(self, thresholds={}, sensor_values=[]):
        self._sensor_values = sensor_values
        if thresholds:
            self._thresholds = thresholds

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
    