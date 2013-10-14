
class Behavior(object):
    """Abstract class for behaivors"""
    def __init__(self, data):
        self.data = data
        print 'I am being initialized: ', self
        
class ObjAvoid(Behavior):
    """Avoid objects like the plague."""

    def __init__(self, data):
        super(ObjAvoid, self).__init__(data)

    def step(self):
        if not self.data.sensor_values:
            print 'no values available'
            return
        vals = self.data.sensor_values
        max_ir_reading = self.data.thresholds['max_ir_reading']
        vl = 5
        vr = 5

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

        return (vl,vr)
        
class WallFollow(Behavior):
    """try to follow a wall as closely as possible"""
    def __init__(self, data):
        super(WallFollow, self).__init__(data)
        self.prev_vals = self.data.sensor_values
        self.avg_l = []
        self.avg_r = []

    def step(self):
        if not self.data.sensor_values:
            print 'no values availablez'
            return
        vals = self.data.sensor_values
        max_ir_reading = self.data.thresholds['max_ir_reading']
        vl = 5
        vr = 5
        #calculate the change relative to the wall since last step
        dl = vals[0] - self.prev_vals[0]
        dr = vals[5] - self.prev_vals[5]
        dl_avg = 0
        dr_avg = 0

        self.avg_l.append(dl)
        self.avg_r.append(dr)

        #Calculate the average of the last three distance changes
        if len(self.avg_l) == 3:
            total = 0
            for val in self.avg_l:
                total += val
            dl_avg = total/3
            total = 0
            for val in self.avg_r:
                total += val
            dr_avg = total/3
            self.avg_l.pop(0) # shift the lists left to make room
            self.avg_r.pop(0)

        else:
            dl_avg = dl
            dr_avg = dr

        '''dl and dr represent the change in 'distance' 
        (in terms of an ir reading) between the robot
        and the wall.
        positive: I've moved further away
        negative: I've moved closer closer
        '''
        if vals[0] > vals[5]: #Wall on the left side
            if dl_avg > 0: #Moving further away (right)
                #print 'turning left'
                vl -= 3
            elif dl_avg < 0:#Moving closer (left)
                #print 'turning right'
                vr -= 3

        elif vals[5] > vals[0]: #Wall on the ride side
            if dr_avg > 0:
                #print 'turning right'
                vr -= 3
            elif dr_avg < 0:
                #print 'turning left'
                vl -= 3

        self.prev_vals = vals
        
        return vl, vr
        