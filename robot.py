'''This is the container module for the pykhepera code. It handles 
high level functions such as reloading the robot, controlling the robot,
etc.
'''
import serial
import pykhepera
import time

import signal, sys


def load():
	reload(pykhepera)
	r = pykhepera.PyKhepera()
	return r

def start(r):
	try:
		max_ir_reading = 100 # This represents the min distance
		prev_l = 5
		prev_r = 5
		r.turn(prev_l,prev_r)
		while(1):
			vl = 5
			vr = 5
			vals = r.get_values('n')
			print 'IR readings: %s' % vals

			if(vals[1] > max_ir_reading):
				vr -= 4
			if(vals[2] > max_ir_reading):
				vr -= 4
			if(vals[3] > max_ir_reading):
				vl -= 4
			if(vals[4] > max_ir_reading):
				vl -= 4
			if((vl == vr) and vl != 5):
				vl = -vl
			if (vl, vr) != (prev_l, prev_r):
				print 'setting speed to: left: %s, right: %s' % (vl, vr)
				r.turn(vl,vr)
				prev_l = vl
				prev_r = vr

	except KeyboardInterrupt:
		print 'killing and cleaning up'
		r.purge_buffer()
		r.stop()
		r.kill()

def normalize_IR(val):
	normalized_val = (float(val)/float(1024))
	return normalized_val