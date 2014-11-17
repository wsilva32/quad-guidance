#!/usr/bin/env python

'''
set stream rate on an APM
'''

import sys, struct, time, os

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--baudrate", type=int,
                  help="master port baud rate", default=115200)
parser.add_argument("--device", required=True, help="serial device")
parser.add_argument("--rate", default=4, type=int, help="requested stream rate")
parser.add_argument("--source-system", dest='SOURCE_SYSTEM', type=int,
                  default=255, help='MAVLink source system for this GCS')
parser.add_argument("--showmessages", action='store_true',
                  help="show incoming messages", default=False)
args = parser.parse_args()

from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module

def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    m.wait_heartbeat()
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))

def show_messages(m):
    '''show incoming mavlink messages'''
    while True:
        msg = m.recv_match(blocking=True)
        if not msg:
            return
        if msg.get_type() == "BAD_DATA":
            if mavutil.all_printable(msg.data):
                sys.stdout.write(msg.data)
                sys.stdout.flush()
        else:
            print(msg)

# create a mavlink serial instance
master = mavutil.mavlink_connection(args.device, baud=args.baudrate)

# wait for the heartbeat msg to find the system ID
wait_heartbeat(master)

#make sure throttle is low before arming
data = [ 0 ] * 8
data[2] = 0  #0 - roll, 1 - pitch, 2 - throttle, 3 - yaw
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

time.sleep(15)	#wait for arm to complete

#move throttle to midstick
data = [ 0 ] * 8
data[2] = 1600
master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

#set mode to auto
master.set_mode('AUTO')


#print("Sending all stream request for rate %u" % args.rate)
#for i in range(0, 3):
    #master.mav.request_data_stream_send(master.target_system, master.target_component,
                                        #mavutil.mavlink.MAV_DATA_STREAM_ALL, args.rate, 1)

if args.showmessages:
    show_messages(master)
