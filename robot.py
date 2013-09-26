'''This is the container module for the pykhepera code. It handles 
high level functions such as reloading the robot, controlling the robot,
etc.
'''
import serial
import pykhepera


def load():
	reload(pykhepera)
	r = pykhepera.PyKhepera()
	return r

