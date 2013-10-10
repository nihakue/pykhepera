class Behavior(object):
    """Abstract class for behaivors"""
    def __init__(self, _sensor_data):
        self._sensor_data = _sensor_data
        print 'I am being initialed: ', self
        
class ObjAvoid(Behavior):
    """docstring for ObjAvoid"""
    def __init__(self, _sensor_data):
        super(ObjAvoid, self).__init__(_sensor_data)
    def step(self):
        print 'avoid'
        print self._sensor_data
        
class WallFollow(Behavior):
    """docstring for WallFollow"""
    def __init__(self, _sensor_data):
        super(WallFollow, self).__init__(_sensor_data)
    def step(self):
        print 'follow'
        