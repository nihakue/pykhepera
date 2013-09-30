'''This is a tiny helper module containing wrapper functions for working 
with the khepera robot
'''

import serial

class PyKhepera():
    """this represents a khepera robot."""

    #Constants
    _get_commands = ['N', 'H', 'O']

    _set_commands = {
        'C': ['pos_left', 'pos_right'],
        'G': ['pos_left', 'pos_right'],
        'L': ['led_num', 'state']
    }

    def __init__(self, port='/dev/ttyS0', baud=9600, timeout=.5):
        self.timeout = timeout
        self.port = port
        self.baud = baud
        self.newlines = ['\r', '\n']
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)

        self.state = 0

        self.purge_buffer()

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

    def turn(self, left, right):
        print 'setting speed to: %s %s' % (left, right)
        self.ser.write('D,%d,%d\n' % (left, right))
        self.purge_buffer()

    def get_values(self, command):
        command = command.upper()
        self.purge_buffer()

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
            return_vals.append(int(val))
        if len(return_vals) == 0:
            raise IndexError("return vals is empty")
        return return_vals

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


