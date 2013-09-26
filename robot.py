'''This is the container module for the pykhepera code. It handles 
high level functions such as reloading the robot, controlling the robot,
etc.
'''
import serial
from pykhepera import PyKhepera


def load_robot():
	reload(pykhepera)
	r = PyKhepera()
	return r

