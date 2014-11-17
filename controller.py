#!/usr/bin/env python

#Imports
import sys, struct, time, os
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module

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

