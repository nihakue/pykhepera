'''This is a tiny helper module containing wrapper functions for working 
with the khepera robot
'''

import serial

class PyKhepera():
    """this represents a khepera robot."""

    #Constants
    _gets = ['N', 'H']

    def __init__(self, port='/dev/ttyS0', baud=9600, timeout=1):
        self.timeout = timeout
        self.port = port
        self.baud = baud
        self.newlines = ['\r', '\n']
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)

        self.purge_buffer()

    def purge_buffer(self):
        response = self.ser.readline()
        while response:
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

    def turn(self, left, right):
        self.ser.write('D,%d,%d\n' % (left, right))
        self.purge_buffer()

    def get_values(self, command):
        command = command.upper()

        if command not in PyKhepera._gets:
            print 'not a valid command'
            return

        self.ser.write('%s\n' % (command))
        sensor_string = self.ser.readline()
        vals = sensor_string.split(',')
        return_vals = []
        for val in vals:
            for token in self.newlines:
                val = val.replace(token,'')
            if val == command.lower():
                continue
            return_vals.append(val)
        return return_vals

    def set_counts(self, left, right):
        self.ser.write('G,%d,%d\n' % (left, right))
        self.purge_buffer()


