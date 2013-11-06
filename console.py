import robot
import time
import sys

r = robot.Robot()

def main():
    running = True
    while running:
        input = raw_input('pykhepera>').split()
        command = input[0].lower()
        args = input[1:]
        resolve(command, args)
    return 0

def resolve(command, *args):
    print command
    if command == 's':
        'running robot'
        r.run()
    if command == 'c':
        print 'calibrating sensors'
        r.calibrate_min()

if __name__ == '__main__':
    result = main()
    sys.exit(result)
