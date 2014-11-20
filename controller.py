#!/usr/bin/env python

#Imports
import sys, struct, time, os
from pymavlink import mavutil
import numpy as np
from math import *
from attitude_tools import angle2dcm
from KF_error import error_derivative_KF
from coord_trans import lla2flatdumb
from target_sim import target_sim
#Define functions
def wait_heartbeat(m):
	'''wait for a heartbeat so we know the target system IDs'''
	print("Waiting for APM heartbeat")
	m.wait_heartbeat()
	print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))

#Program Start

#TARGET INFO(for SiL only)#
#desired coordinate center (datum)
lat0 = 40.1447601
lon0 = -105.2435532
alt0 = 1680.38

t_pos = np.array([[100], [30], [10]])
#NOTE: need a way to ensure we are pointed towars the target before begining so we can see it in the image (maybe just command a reasonably close yaw?)


#GAINS#
#throttle
KP_t = 1
KI_t = 0
KD_t = 0

#pitch
KP_p = 0.05
KI_p = 0
KD_p = 0

#body z rotation rate
KP_r = 1
KI_r = 0
KD_r = 0

#roll
KP_rl = 1
KI_rl = 0
KD_rl = 0

#SATURATION LIMITS#
roll_sat = 20
pitch_sat = 20
r_sat = 2
throttle_sat = 1

#TEMP!!!
target = np.array([1,2])

#PARAMETERS
g = 9.807;
m = 2;
#level hover thrust fraction
th0 = 0.3;

#max thrust

#field of view from center in degrees
FoVh = 40*pi/180
FoVv = 40*pi/180

#field of view in pixels from center
FoVph = 320
FoVpv = 320

#range to virtual plane in pixels
rangeh = FoVph/tan(FoVh)
rangev = FoVpv/tan(FoVv)
start = True
prev = time.time()
while True:

	#get the time step
	dt = time.time() - prev
	prev = time.time()
	#Get velocity (vel), roll and pitch
	roll = msg.roll
	pitch = msg.pitch

	#get the target position in the image
	
	# get the position and yaw (needed only for SiL, may be needed later for search phase but that should come from Vicon)
	yaw = msg.yaw
	
	lat = msg.lat/(10**7)
	lon = msg.lon/(10**7)
	alt = msg.alt/1000
	
 	pos = lla2flatdumb(lat,lon,alt,lat0,lon0,alt0)
	x = pos[0]
	y = pos[1]
	z = pos[2]
	
	# simulate the target for SiL
	
	target_sim(t_pos,x,y,z,roll,pitch,yaw,FoVv, FoVh, FoVpv, FoVph)
	
	#MEASUREMENT CONVERSION# (target pixels to desired flight path and yaw difference)

	#z down
	el = atan2(-target[2], rangev)
	az = atan2(target[1], rangeh)

	#vector to the virtual plane point
	vecb = np.array([1], [tan(az)], [tan(el)])

	#normalize
	vecb = vecb/np.linalg.norm(vecb)

	#we are interested in relative yaw angle so the yaw can be assumed zero
	Rib_image = angle2dcm(0, pitch, roll)
	Rbi_image = np.transpose(Rib_image)

	vec = np.dot(Rbi_image,vecb)

	#desired flightpath angle (positive is below horizon)
	gam_d = np.arctan2(vec[3],vec[1])
	#yaw difference (positve is to the right of the vehicle)
	yaw_diff = np.arctan2(vec[2],vec[1])

	#the controller error quantities are the yaw_diff and gam_d

	#CONTROLLER OVERVIEW

	#in general we want to yaw toward the target and climb up to it
	#a speed control is also needed for image tracking
	#Proposed design

	#roll is set to prevent sideways movement (no sideways motion should be
	#needed or helpful here) needs to be kept minimal though

	#should maybe also try simplified system with simply setting roll at zero

	#speed control loop acts on pitch angle

	#altitude hold uses throttle with feed forward term from known pitch, roll and
	#weight (ie the global z component must counter gravity) 

	#rotation rate (r) is set with PID (D may not be possible) control to the
	#yaw difference (yaw_diff) with the assumption that the pitch is small
	#(may want to investigate a better method later)


	#ALTITUDE CONTROL#
	#differentiator
	P_bar_0 = np.diag(np.array([0.01, 1])^2)
	x_bar0 = np.transpose(np.array([gam_d, 0]))
	Q = np.diag(np.array([0.01, 0.3])^2)
	R = 1*pi/180
	if start:
		KF_gam = error_derivative_KF(start,P_bar_0,x_bar0,dt,Q,R,gam_d)
	else:
		KF_gam = error_derivative_KF(start,P_bar_0,x_bar0,dt,Q,R,gam_d,Pk_gam,xk_gam)
	Pk_gam = KF_gam[1]
	xk_gam = KF_gam[2]
	gam_dot = xk_gam[2]
	#integrator
	gam_int = gam_int + gam_d*dt

	#feed-forward
	throttle_com_ff = th0/(cos(roll)*cos(pitch))

	#we may need P and I control here
	throttle_com = throttle_com_ff - KP_t*gam_d - KI_t*gam_int - KD_t*gam_dot

	#throttle saturation
	if throttle_com > throttle_sat:
		throttle_com = throttle_sat
	else if throttle_com < 0:
		throttle_com = 0

	#SPEED CONTROL#

	#given no knowledge of drag coefficients on the aircraft take the simple
	#gain tuning approach with the concept that zero pitch gives zero speed in
	#the unperturbed case

	#forward speed from state
	#with GPS/Vicon type measurements convert this to just use the horizontal velocity in the inertial frame
	vel = np.dot(Rbi_image,np.array([x[4]], [x[5]], [x[6]]))
	speed = vel[1]

	speed_err = speed - speed_des

	#differentiator
	P_bar_0 = np.diag(np.array([0.1, 1])^2)
	x_bar0 = np.transpose(np.array([speed_err, 0]))
	Q = np.diag(np.array([0.01, 0.1])^2)
	R = 0.1
	if start:
		KF_speed_err = error_derivative_KF(start,P_bar_0,x_bar0,dt,Q,R,speed_err)
	else:
		KF_speed_err = error_derivative_KF(start,P_bar_0,x_bar0,dt,Q,R,speed_err,Pk_speed_err,xk_speed_err)
	Pk_speed_err = KF_speed_err[1]
	xk_speed_err = KF_speed_err[2]
	speed_err_dot = xk_speed_err[2]
	#integrator
	speed_err_int = speed_err_int + speed_err*dt

	#pitch down speeds you up
	pitch_com = KP_p*speed_err + KI_p*speed_err_int + KD_p*speed_err_dot

	#pitch saturation
	if pitch_com > pitch_sat:
		pitch_com = pitch_sat
	else if pitch_com < -pitch_sat:
		pitch_com = -pitch_sat;

	#YAW CONTROL#

	#differentiator
	P_bar_0 = np.diag(np.array([0.1, 1])^2)
	x_bar0 = np.transpose(np.array([yaw_diff, 0]))
	Q = np.diag(np.array([0.01, 0.3])^2)
	R = 1*pi/180
	if start:
		KF_speed_err = error_derivative_KF(start,P_bar_0,x_bar0,dt,Q,R,yaw_diff)
	else:
		KF_yaw_diff = error_derivative_KF(start,P_bar_0,x_bar0,dt,Q,R,yaw_diff,Pk_yaw_diff,xk_yaw_diff)
	Pk_yaw_diff = KF_yaw_diff[1]
	xk_yaw_diff = KF_yaw_diff[2]
	yaw_diff_dot = xk_yaw_diff[2]
	#integator
	yaw_diff_int = yaw_diff_int + yaw_diff*dt

	#command
	r_com = KP_r*yaw_diff + KI_r*yaw_diff_int + KD_r*yaw_diff_dot

	#r saturation
	if r_com > r_sat:
		r_com = r_sat
	else if r_com < -r_sat:
		r_com = -r_sat

	#ROLL CONTROL#

	#sideways motion
	sw_slip = vel[2]

	#differentiator
	P_bar_0 = np.diag(np.array([0.1, 1])^2)
	x_bar0 = np.transpose(np.array([sw_slip, 0]))
	Q = np.diag(np.array([0.01, 0.3])^2)
	R = 0.1
	if start:
		KF_speed_err = error_derivative_KF(start,P_bar_0,x_bar0,dt,Q,R,sw_slip)
	else:
		KF_yaw_diff = error_derivative_KF(start,P_bar_0,x_bar0,dt,Q,R,sw_slip,Pk_sw_slip,xk_sw_slip)
	Pk_sw_slip = KF_sw_slip[1]
	xk_sw_slip = KF_sw_slip[2]
	sw_slip_dot = xk_sw_slip[2]
	#integrator
	sw_slip_int = sw_slip_int + sw_slip*dt

	#command (positive roll -> positive slip)
	roll_com = -KP_rl*sw_slip -KI_rl*sw_slip_int -KD_rl*sw_slip_dot

	#roll saturation
	if roll_com > roll_sat:
		roll_com = roll_sat
	else if roll_com < -roll_sat:
		roll_com = -roll_sat

	#COMMAND#
	u = [pitch_com roll_com r_com throttle_com]
	start = False



















# create a mavlink serial instance
master = mavutil.mavlink_connection('udp:127.0.0.1:14551', baud=115200)

# wait for the heartbeat msg to find the system ID
wait_heartbeat(master)

#Download state information
while True:
	msg = master.recv_match(type='ATTITUDE',blocking=False)
	if not msg is None:
		print "Roll: %f" % msg.roll
		print "Pitch: %f" % msg.pitch
		print "Yaw: %f" % msg.yaw
		print "Rollspeed: %f" % msg.rollspeed
		print "Pitchspeed: %f" % msg.pitchspeed
		print "Yawspeed: %f" % msg.yawspeed

	msg = master.recv_match(type='GLOBAL_POSITION_INT',blocking=False)
	if not msg is None:
		print "Latitude: %f" % (msg.lat/(10**7))
		print "Longitude: %f" % (msg.lon/(10**7))
		print "Altitude: %f" % (msg.alt/1000)

#make sure throttle is low before arming
data = [ 0 ] * 8
data[2] = 0  #index => 0 = roll, 1 = pitch, 2 = throttle, 3 = yaw
master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

#set mode to stabilize
master.set_mode('STABILIZE')

#Set arming check to 0
master.param_set_send('ARMING_CHECK',0)

#disable DCM Threshold check
master.param_set_send('DCM_CHECK_THRESH',0)
#http://copter.ardupilot.com/wiki/ekf-inav-failsafe/

#Arm ArduCopter
master.arducopter_arm()

time.sleep(15)	#delay to wait for arm to complete

#move throttle to 1600
data = [ 0 ] * 8
data[2] = 1600
master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

#set mode to auto
master.set_mode('AUTO')

