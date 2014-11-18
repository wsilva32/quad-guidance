#!/usr/bin/env python

#Imports
import sys, struct, time, os
from pymavlink import mavutil
import numpy as np
from math import *
from attitude_tools import angle2dcm


#Define functions
def wait_heartbeat(m):
	'''wait for a heartbeat so we know the target system IDs'''
	print("Waiting for APM heartbeat")
	m.wait_heartbeat()
	print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))

#Program Start

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

#field of view from center in degrees
FoVh = 40*pi/180
FoVv = 40*pi/180

#field of view in pixels from center
FoVph = 320
FoVpv = 320

#range to virtual plane in pixels
rangeh = FoVph/tan(FoVh)
rangev = FoVpv/tan(FoVv)

while True:
	#Get velocity (vel) and roll and pitch
	roll = msg.roll
	pitch = msg.pitch

	#MEASUREMENT CONVERSION# (target pixels to desired flight path and yaw difference)

	#z down
	el = atan2(-target[2], rangev)
	az = atan2(target[1], rangeh)

	#vector to the virtual plane point
	vecb = np.array([1], [tan(az)], [tan(el)])

	#normalize
	vecb = vecb/np.linalg.norm(vecb)

	#we are interested in relative yaw angle so the yaw can be assumed zero
	Rib_image = angle2dcm(0, 
pitch, roll)
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

	#integrator
	gam_int = gam_int + gam_d*dt

	#feed-forward
	throttle_com_ff = m*g/(cos(roll)*cos()*T_max)

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
