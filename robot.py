'''This is the container module for the pykhepera code. It handles 
high level functions such as reloading the robot, controlling the robot,
etc.
'''
import serial
import pykhepera
import time
import math

import signal, sys



def restart():
	r = load()
	start(r)

def load():
	reload(pykhepera)
	r = pykhepera.PyKhepera()
	return r


def start(r):
	try:
		max_ir_reading = 100 # This represents the min distance
		wall_max = 200
		wall_min = 80
		prev_l = 5
		prev_r = 5
		r.turn(prev_l,prev_r)
		prev_vals = r.get_values('n')
		r.state = 0

		avg_l = [] # average change in distances for wall following
		avg_r = []

		while(1):
			print 'state: ', r.state,
			vl = 5
			vr = 5
			vals = r.get_values('n')
			print 'IR readings: %s' % vals
			

			if r.state is 0:
				r.led(1, 0)
				r.led(0, 1)
				#Reset values
				avg_l = []
				avg_r = []
				close = False

				for val in vals[2:3]:
					if val > max_ir_reading:
						close = True

				if close:
					
					print 'i am close'

					if(vals[1] > max_ir_reading/1.5):
						vr -= 4
					if(vals[2] > max_ir_reading):
						vr -= 4
					if(vals[3] > max_ir_reading):
						vl -= 4
					if(vals[4] > max_ir_reading/1.5):
						vl -= 4
					if((vl == vr) and vl != 5): #May never enter this block
						vl = -vl

				if (vl, vr) != (prev_l, prev_r):
					if vl == 5 and vr == 5:
						r.state = 2
						vl = 0
						vr = 0

					r.turn(vl,vr)

			elif r.state is 1: #Align to the wall
				vl = 0
				vr = 0
				r.turn(0,0)
				acceptable_error = 10
				# print 'target val: %.5f' % target_val

				lv = float(vals[1])/float(vals[0])
				rv = float(vals[5])/float(vals[4])

				# print 'left value: %.2f right value: %.2f' % (lv, rv)

				l_error = math.fabs(lv - target_val)
				r_error = math.fabs(rv - target_val)

				# print l_error, r_error

				# if vals[0] > vals[5]: #closer to the right wall
				# 	if l_error > acceptable_error:
				# 		if vals[0] < vals[1]/(2**(1/2)):
				# 			vr -= 2
				# 			vl += 2
				# 		else:
				# 			vl -= 2
				# 			vr += 2
				# 	else:
				# 		r.state = 2
				# 		break
				# else: #closer to the left wall
				# 	if r_error > acceptable_error:
				# 		if vals[5] < vals[4]/(2**(1/2)):
				# 			vl -= 2
				# 			vr += 2
				# 		else:
				# 			vr -= 2
				# 			vr += 2
				# 	else:
				# 		r.state = 2
				# 		break

				# r.turn(vl, vr)


			elif r.state is 2: #Follow the wall
				r.led(0, 1)
				r.led(1, 1)
				prev_vals = r.get_values('n')

				dl = vals[0] - prev_vals[0]
				dr = vals[5] - prev_vals[5]
				dl_avg = 0
				dr_avg = 0

				avg_l.append(dl)
				avg_r.append(dr)

				dl_avg = dl
				dr_avg = dr

				# if len(avg_l) == 3:
				# 	total = 0
				# 	for val in avg_l:
				# 		total += val
				# 	dl_avg = total/3
				# 	total = 0
				# 	for val in avg_r:
				# 		total += val
				# 	dr_avg = total/3
				# 	print 'list left: ', avg_l
				# 	avg_l.pop(0) # shift the lists left to make room
				# 	avg_r.pop(0)

				# else:
				# 	dl_avg = dl
				# 	dr_avg = dr

					# print 'average dl: %d average dr: %d' % (dl_avg, dr_avg)


				if (vals[2] > max_ir_reading) or (vals[3] > max_ir_reading):
					r.state = 0
					print "wall in front"
					continue

				'''dl and dr represent the change in 'distance' 
				(in terms of an ir raeding) between the robot
				and the wall.
				positive: further away
				negative: closer
				'''
				if vals[0] > vals[5]:
					if dl_avg > 0:
						print 'further from left'
						vl -= 4
					elif dl_avg < 0:
						print 'closer to left'
						vr -= 4

				elif vals[5] > vals[0]:
					if dr_avg > 0:
						print 'further from right'
						vr -= 4
					elif dr_avg < 0:
						print 'closer to right'
						vl -= 4

				else:
					r.state = 0
					print "too far from wall"
					vl = 5
					vr = 5
					

				r.turn(vl, vr)

			prev_l = vl
			prev_r = vr
			prev_vals = vals


	except KeyboardInterrupt:
		print 'killing and cleaning up'
		r.purge_buffer()
		r.stop()
		r.kill()

r = load()