'''This is a tiny helper module containing wrapper functions for working
with the khepera robot
'''
import serial

class PyKhepera():
    """this represents a khepera robot."""

    #Constants
    _get_commands = ['N', 'H', 'O', 'E']

    _set_commands = {
        'C': ['pos_left', 'pos_right'],
        'G': ['pos_left', 'pos_right'],
        'L': ['led_num', 'state']
    }


    def __init__(self, port='/dev/ttyS0', baud=9600, timeout=.05):
        self.timeout = timeout
        self.port = port
        self.baud = baud
        self.newlines = ['\r', '\n']
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)

        self.state = 0

        self.purge_buffer()
        self.speed = (0,0)

        self.diameter = 53 #milimeters

    def purge_buffer(self, verbose=False):
        response = self.ser.readline()
        while response:
            if verbose:
                print response
            response = self.ser.readline()

    def read_array(self):
        pass

    def kill(self):
        self.ser.close()
        print 'serial comm stopped'

    def set_speed(self, speed):
        self.ser.write('D,%d,%d\n' % (speed, speed))
        self.purge_buffer()

    def stop(self):
        self.set_speed(0)

    def turn(self, (left, right)):
        if (left,right) != self.speed:
            # print 'setting speed to: %s %s' % (left, right)
            self.ser.write('D,%d,%d\n' % (left, right))
            self.purge_buffer()
            self.speed = left,right

    def read_sensor_values(self):
        return self.get_values('N')

    def read_wheel_values(self):
        return self.get_values('H')

    def reset_wheel_counters(self):
        self.set_values('g', [0, 0])

    def read_wheel_speeds(self):
        return self.get_values('E')

    def get_values(self, command):
        command = command.upper()
        if command not in PyKhepera._get_commands:
            print 'not a valid command'
            return

        self.ser.write('%s\n' % (command)) #Send the command

        sensor_string = self.ser.readline()
        vals = sensor_string.split(',')

        return_vals = []
        for val in vals:
            for token in self.newlines:
                val = val.replace(token,'')
            if val == command.lower():
                continue
            try:
                return_vals.append(int(val))
            except Exception, e:
                continue
        return return_vals

    def to_mm(self, wheel_value):
        return float(wheel_value * 0.08)

    def to_wu(self, mm):
        return int(mm/0.08)

    def rotate(self, rotation, radians=True):
        self.stop()
        current_wv = self.read_wheel_values()
        if radians:
            omega = self.to_wu((self.diameter/2) * rotation)
        desired_wv = [int(current_wv[0] - omega),
            int(current_wv[1] + omega)]

        self.set_values('C', desired_wv)

    def set_values(self, command, args, verbose=False):
        command = command.upper()
        if command not in PyKhepera._set_commands:
            print 'invalid command'
            return
        if len(args) != len(PyKhepera._set_commands[command]):
            print 'invalid arguments.\
            was expecting the form: %s' % PyKhepera._set_commands[command]
            return
        arg_string = ','.join(str(x) for x in args)
        if verbose:
            print 'sending: %s,%s\n' % (command, arg_string)

        self.ser.write('%s,%s\n' % (command, arg_string))
        self.purge_buffer()

    def led(self, led_num, state):
        self.set_values('L', [led_num, state])

    def set_counts(self, left, right):
        self.ser.write('G,%d,%d\n' % (left, right))
        self.purge_buffer()
