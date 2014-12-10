#!/usr/bin/env python
import sys, struct, time, os, signal
import numpy as np
from math import *
import camera.BalloonFinder

#Define functions
def signal_handler(signal, frame):
    global v
    print('Exiting controller')
    v.stop()
    cam.stop()
    sys.exit(0)

cage_len = 5.8 # length of cage side in meters
# DEFINE SEARCH PATTERN
pi = np.pi
angles = np.array([0,45,90,135,180,225,270,310])
locations = np.array([[0, 0],[cage_len/4, cage_len/4],[-cage_len/4, cage_len/4],[-cage_len/4, -cage_len/4],[cage_len/4, -cage_len/4]])
z_d = 1; # desired altitude of 1 meter
loc_tol = 0.5; # distance to consider quad on target in meters
yaw_tol = 10*pi/180; 
print angles
print locations[1]
#GAINS#
#throttle
KP_t = 0.25
KI_t = 0
KD_t = 0.2

#pitch
KP_p = 0.35
KI_p = 0.3
KD_p = 0.08

#body z rotation rate
KP_r = 3
KI_r = 0
KD_r = 0

#roll
KP_rl = 2.3
KI_rl = 0
KD_rl = 0.2

#CONTROLLER SATURATION LIMITS#
roll_sat = 20*pi/180
pitch_sat = 20*pi/180
r_sat = 45*pi/180
throttle_sat = 1

#PARAMETERS
g = 9.807;
m = 2;
#level hover thrust fraction
th0 = 0.6;

#RC CHANNEL LIMITS (NEED TO CHANGE HOW WE GET THESE)


RC1_MAX = 1925.000000
RC1_MIN =  1106.000000
RC1_ZERO = 1500;

RC2_MAX = 1930.000000
RC2_MIN = 1110.000000
RC2_ZERO = 1500;

RC3_MAX = 1929.000000
RC3_MIN = 1110.000000
RC3_ZERO = 1500;

RC4_MAX = 1933.000000
RC4_MIN = 1109.000000
RC4_ZERO = 1500;


#INPUT SATURATION LIMITS
LIM_PITCH_MIN = -4500.00000/100 * pi/180;
LIM_PITCH_MAX = 4500.00000/100 * pi/180;

LIM_ROLL = 4500.00000/100 * pi/180;

LIM_YAW_RATE = 4.500000*200/4.5 * pi/180;

#max thrust

#field of view from center in degrees
FoVh = 27*pi/180
FoVv = 20*pi/180

#field of view in pixels from center
FoVph = 320
FoVpv = 320

#range to virtual plane in pixels

start = True
prev = time.time()

#Initialize camera balloon tracking routine
#cam = 0
#signal.signal(signal.SIGINT, signal_handler)
#cam = camera.BalloonFinder.BalloonFinder(640,480)
#cam.start()
#print 'Current video size: %dx%d' % (cam.vidSize[0], cam.vidSize[1])
#print 'Area: %d Centroid: (%d,%d) FPS: %1.2f' % (cam.area, cam.centroid[0], cam.centroid[1], cam.frameRate)

#Initialize vicon system
v=0
#v = Position.ViconPosition(vicon_object + "@192.168.20.10")
position = np.array([0,0,0])
#v = FakePosition.FakePosition(1.2, 1.5, 1.4)

#v.start()
#print 'Z: %1.4f' % v.position[2]

time.sleep(5)
getMilliTime = lambda: int(round(time.time() * 1000))
roll = 0;
pitch = 0;

# counters
angi = 0;
loci = 0;
prev = 0;

tracking = True
# main loop
while True:
	chi_d = angles[angi]
	loc_d = locations[loci]
	x_d = loc_d[0]
	y_d = loc_d[1]
	

	x = position[0]
	y = position[1]
	z = position[2]
	print z
	# get delta t
	dt= time.time() - prev
	prev = time.time()

	alt_err = z_d - z
	x_err = x_d - x
	y_err = y_d - y
	loc_err = np.sqrt(x_err**2 + y_err**2)

	# ALITUDE CONTROLLER

	
	# proportional
	alt_err = z - z_d
	
	# integral
	if start:
		alt_int = 0;
	alt_int += alt_err*dt

	# derivative
	alt_dot = alt_err/dt

	# throttle commands
	throttle_com_ff = th0/(cos(roll)*cos(pitch))
	throttle_com = throttle_com_ff  - KP_t*alt_err - KI_t*alt_int - KD_t*alt_dot

	if throttle_com > throttle_sat:
		throttle_com = throttle_sat
	elif throttle_com < 0:
		throttle_com = 0

	# check if quad is within acceptable dist of target
	if (loc_err > loc_tol): # not near desired location
		yaw_err = chi - atan2(x_err,y_err)
		if yaw_err > yaw_tol:
			orienting = True
			searching = False
	else:
		orienting = False
		searching = True
		yaw_err = chi - angles[angi]
		
		
	
	# YAW RATE CONTROLLER
	# errors

	if yaw_err < yaw_tol:
		if orienting:
			orienting = False
			moving = True
		elif searching:
			angi += 1
			if angi > 7:
				angi = 0;
				searching = False
				orienting = True
			
		
	if start:
		yaw_int = 0
	yaw_int += yaw_err*dt

	

	yaw_err_dot = 0
	r_com = KP_r*yaw_err + KI_r*yaw_int + KD_r*yaw_err_dot

	#r saturation
	if r_com > r_sat:
		r_com = r_sat
	elif r_com < -r_sat:
		r_com = -r_sat

	# POSITION CONTROL

	# if not moving, 
	if searching | orienting:
		loc_err = -pitch
	if start:
		loc_int = 0
	loc_int += loc_err*dt
	#loc_dot = loc_err/dt
	loc_dot = 0
	loc_int = 0
	pitch_com = KP_p*loc_err + KI_p*loc_int + KD_p*loc_dot



	#ROLL CONTROL#

	#sideways motion
	sw_slip = vel[2]


	#differentiator
	#print np.power(np.array([0.01, 1]),2)
	P_bar_0 = np.diag(np.power(np.array([0.1, 1]),2))
	x_bar0 = np.concatenate([sw_slip, np.array([0])]).reshape(2,1)
	#print x_bar0
	Q = np.diag(np.power(np.array([0.01, 0.3]),2))
	R = 0.1
	if start:
		KF_sw_slip = error_derivative_KF(start,P_bar_0,x_bar0,dt,Q,R,sw_slip)
	else:
		KF_sw_slip = error_derivative_KF(start,P_bar_0,x_bar0,dt,Q,R,sw_slip,Pk_sw_slip,xk_sw_slip)
	Pk_sw_slip = KF_sw_slip[1]
	xk_sw_slip = KF_sw_slip[0]
	sw_slip_dot = xk_sw_slip[1]
	#    print xk_sw_slip
	#integrator
	if start:
		sw_slip_int = 0;
	sw_slip_int += sw_slip*dt

	#	#differentiator
	#	P_bar_0 = np.diag(np.array([0.1, 1])^2)
	#	x_bar0 = np.transpose(np.array([sw_slip, 0]))
	#	Q = np.diag(np.array([0.01, 0.3])^2)
	#	R = 0.1
	#	if start:
	#		KF_speed_err = error_derivative_KF(start,P_bar_0,x_bar0,dt,Q,R,sw_slip)
	#	else:
	#		KF_yaw_diff = error_derivative_KF(start,P_bar_0,x_bar0,dt,Q,R,sw_slip,Pk_sw_slip,xk_sw_slip)
	#	Pk_sw_slip = KF_sw_slip[1]
	#	xk_sw_slip = KF_sw_slip[0]
	#	sw_slip_dot = xk_sw_slip[1]
	#	#integrator
	#	sw_slip_int = sw_slip_int + sw_slip*dt

	#command (positive roll -> positive slip)
	roll_com = -KP_rl*sw_slip -KI_rl*sw_slip_int -KD_rl*sw_slip_dot

	#roll saturation
	if roll_com > roll_sat:
		roll_com = roll_sat
	elif roll_com < -roll_sat:
		roll_com = -roll_sat

	u = [pitch_com, roll_com, -r_com, throttle_com]
		#
