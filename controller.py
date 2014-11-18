#!/usr/bin/env python

#Imports
import sys, struct, time, os
from pymavlink import mavutil

#Define functions
def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    m.wait_heartbeat()
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))

#Program Start

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

