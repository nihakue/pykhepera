import robot
import time
import sys

def main():
    running = True
    while running:
        input = raw_input('pykhepera>').split()
        command = input[0]
        args = input[1:]
        resolve(command, args)
    return 0

def resolve(command, *args):
    for arg in args:
        print arg

if __name__ == '__main__':
    result = main()
    sys.exit(result)
