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
		max_wall_reading = 60
		prev_l = 5
		prev_r = 5
		r.turn(prev_l,prev_r)
		prev_vals = r.get_values('n')

		avg_l = [] # average change in distances for wall following
		avg_r = []

		while(1):
			print 'state: ', r.state
			vl = 5
			vr = 5
			vals = r.get_values('n')
			print 'IR readings: %s' % vals

			if r.state is 0:

				if(vals[1] > max_ir_reading):
					vr -= 4
				if(vals[2] > max_ir_reading):
					vr -= 4
				if(vals[3] > max_ir_reading):
					vl -= 4
				if(vals[4] > max_ir_reading):
					vl -= 4
				# if((vl == vr) and vl != 5):
				# 	vl = -vl
				if (vl, vr) != (prev_l, prev_r):
					r.turn(vl,vr)
					prev_l = vl
					prev_r = vr
					if (vl, vr) == (5, 5):
						r.state = 1

			elif r.state is 1:
				dl = vals[0] - prev_vals[0]
				dr = vals[5] - prev_vals[5]
				dl_avg = 0
				dr_avg = 0

				avg_l.append(dl)
				avg_r.append(dr)
				
				if len(avg_l) == 3:
					total = 0
					for val in avg_l:
						total += val
					dl_avg = total/3
					total = 0
					for val in avg_r:
						total += val
					dr_avg = total/3

					avg_l.pop(0) # shift the lists left to make room
					avg_r.pop(0)

					print 'average dl: %d average dr: %d' % (dl_avg, dr_avg)

				if vals[2] > max_ir_reading or vals[3] > max_ir_reading:
					r.state = 0
					continue

				if vals[0] > max_wall_reading:
					if dl_avg > 0:
						vr -= 1
					else:
						vl -= 1

				if vals[5] > max_wall_reading:
					if dr_avg > 0:
						vl -= 1
					else:
						vr -= 1

					r.turn(vl, vr)

				else:
					r.state = 0
					r.turn(5, 5)
					continue

			prev_vals = vals


	except KeyboardInterrupt:
		print 'killing and cleaning up'
		r.purge_buffer()
		r.stop()
		r.kill()

def normalize_IR(val):
	normalized_val = (float(val)/float(1024))
	return normalized_val