import multiprocessing
import serial

class Termi(multiprocessing.Process):
    """Listens to the serial communication in its own thread"""
    def __init__(self, serial_object):
        super(Termi, self).__init__()
        self.ser = serial_object
        self.exit_flag = 0
        self.daemon = True
        
    def run(self):
        if not self.ser:
            'TERMINAL creating its own serial object'
            self.ser = serial.Serial('/dev/ttyS0', 9600)
        while not self.exit_flag:
            s = self.ser.read(1)
            if s:
                print s
                for line in self.ser.readlines():
                    print line
        print 'I am exiting: ', self