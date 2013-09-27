'''This is the container module for the pykhepera code. It handles 
high level functions such as reloading the robot, controlling the robot,
etc.
'''
import serial
import pykhepera


def load():
	mind = 60 ''' This is the minimum distance to turn away from an object'''
	reload(pykhepera)
	r = pykhepera.PyKhepera()
	while(1):
		vl = 5
		vr = 5
		val = get_values(n)
		if(val[1] < mind):
			--vr
		if(val[2] < mind):
			--vr
		if(val[3] < mind):
			--vl
		if(val[4] < mind):
			--vr
		if(vl == vr):
			vl = -vl
		r.set_speed(vl,vr)
	return r

