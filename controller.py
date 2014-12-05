#!/usr/bin/env python

#Imports
import sys, struct, time, os, signal
from pymavlink import mavutil
import numpy as np
from math import *
from attitude_tools import angle2dcm
from KF_error import error_derivative_KF
from velocity_KF import velocity_KF
from coordtrans import lla2flatdumb
#from target_sim import target_sim
#import Position
#Define functions
def signal_handler(signal, frame):
    global v
    print('Exiting controller')
    v.stop()
    sys.exit(0)

def wait_heartbeat(m):
	'''wait for a heartbeat so we know the target system IDs'''
	print("Waiting for APM heartbeat")
	m.wait_heartbeat()
	print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))

#Program Start
# create a mavlink serial instance
#master = mavutil.mavlink_connection('udp:127.0.0.1:14551', baud=115200)

# wait for the heartbeat msg to find the system ID
#wait_heartbeat(master)

#Setup Vicon connection
#v = 0
#signal.signal(signal.SIGINT, signal_handler)
#v = Position.ViconPosition("Wand2@192.168.20.10")
#v.start()

#TARGET INFO(for SiL only)#
#desired coordinate center (datum)
lat0 = 40.1447601
lon0 = -105.2435532
alt0 = 1680.38

t_pos = np.array([[100], [30], [10]])
#NOTE: need a way to ensure we are pointed towars the target before begining so we can see it in the image (maybe just command a reasonably close yaw?)

#target speed
speed_des = 1
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
KI_r = 1
KD_r = 0

#roll
KP_rl = 2.3
KI_rl = 0
KD_rl = 0.2

#CONTROLLER SATURATION LIMITS#
roll_sat = 20
pitch_sat = 20
r_sat = 2
throttle_sat = 1

##TEMP!!!
#target = np.array([1,2])

#PARAMETERS
g = 9.807;
m = 2;
#level hover thrust fraction
th0 = 0.3;

#RC CANNEL LIMITS (NEED TO CHANGE HOW WE GET THESE)
RC1_MAX = v.parameters['RC1_MAX']
RC1_MIN = v.parameters['RC1_MIN']
RC1_ZERO = 1500;

RC2_MAX = v.parameters['RC2_MAX']
RC2_MIN = v.parameters['RC2_MIN']
RC2_ZERO = 1500;

RC3_MAX = v.parameters['RC3_MAX']
RC3_MIN = v.parameters['RC3_MIN']
RC3_ZERO = 1500;

RC4_MAX = v.parameters['RC4_MAX']
RC4_MIN = v.parameters['RC4_MIN']
RC4_ZERO = 1500;

#INPUT SATURATION LIMITS
LIM_PITCH_MIN = v.parameters['LIM_PITCH_MIN']
LIM_PITCH_MAX = v.parameters['LIM_PITCH_MAX']

LIM_ROLL  = v.parameters['LIM_ROLL_CD']

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
gam_target = 0;

while True:

#	master.mav.request_data_stream_send(master.target_system, master.target_component,mavutil.mavlink.MAV_DATA_STREAM_ALL, 25, 1)
#	msg = master.recv_match(type='ATTITUDE',blocking=False)
#
    #get the time step
    dt = time.time() - prev
    prev = time.time()
#	#Get velocity (vel), roll and pitch (roll and pitch from aircraft, velocity from filtering vicon)
#	roll = msg.roll
#	pitch = msg.pitch
#	yaw = v.angles[2] #verify what order angles is in
#	
#	#position from vicon (invert y and z to flip into z down coordinate frame)
#    pos_vic_fence = pos_vic
#    pos_vic[1] = -pos_vic[1]
#    pos_vic[2] = -pos_vic[2]

    #print pos_vic

    #simulate vicon and IMU data
    roll = 0
    pitch = 0
    yaw = 0
    pos_vic = np.array([0,0,0])

    #could all be moved ahead of the loop
    R_vic = np.identity(3)*0.001
    Q_vic = np.identity(6)*0.1
    P_bar_0_vic = np.identity(6)
    x_bar0_vic = np.concatenate([pos_vic, np.array([0, 0,0])])
    x_bar0_vic = x_bar0_vic.reshape((6, 1))
    #estimate the velocity (would like tomove this to a paralell operation at higher rate, will need dt calculated in that loop as well)
    if start:
        vkf = velocity_KF(start,P_bar_0_vic,x_bar0_vic,dt,Q_vic,R_vic,pos_vic.reshape((3,1)))
        #print x_bar0_vic
    else:
        vkf = velocity_KF(start,P_bar_0_vic,x_bar0_vic,dt,Q_vic,R_vic,pos_vic.reshape((3,1)),Pk_vic,x_vic)

    Pk_vic = vkf[1]
    x_vic = vkf[0]
    vel_i = x_vic[0:3]
    #print x_vic
    #print vel_i
    #rotate inertial frame velocity to aircraft frame
    Rib = angle2dcm(yaw,roll,pitch)
    vel = np.dot(Rib,vel_i)
    #get the target position in the image
    target = np.array([50,  -100])
    ## get the position and yaw (needed only for SiL, may be needed later for search phase but that should come from Vicon)
    #yaw = msg.yaw
    #
    #lat = msg.lat/(10**7)
    #lon = msg.lon/(10**7)
    #alt = msg.alt/1000

    #pos = lla2flatdumb(lat,lon,alt,lat0,lon0,alt0)
    #x = pos[0]
    #y = pos[1]
    #z = pos[2]

    # simulate the target for SiL

    #target_sim(t_pos,x,y,z,roll,pitch,yaw,FoVv, FoVh, FoVpv, FoVph)

    #MEASUREMENT CONVERSION# (target pixels to desired flight path and yaw difference)

    #z down
    el = atan2(-target[1], rangev)
    az = atan2(target[0], rangeh)

    #vector to the virtual plane point
    vecb = np.array([[1], [tan(az)], [tan(el)]])

    #normalize
    vecb = vecb/np.linalg.norm(vecb)

    #we are interested in relative yaw angle so the yaw can be assumed zero
    Rib_image = angle2dcm(0, pitch, roll)
    Rbi_image = np.transpose(Rib_image)

    vec = np.dot(Rbi_image,vecb)

    #desired flightpath angle (positive is below horizon)
    gam_d = np.arctan2(vec[2],vec[0])
    #yaw difference (positve is to the right of the vehicle)
    yaw_diff = np.arctan2(vec[1],vec[0])

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
    #print np.power(np.array([0.01, 1]),2)
    P_bar_0 = np.diag(np.power(np.array([0.01, 1]),2))
    x_bar0 = np.concatenate([gam_d, np.array([0])]).reshape(2,1)
    #print x_bar0
    Q = np.diag(np.power(np.array([0.01, 0.3]),2))
    R = 1*pi/180
    if start:
        KF_gam = error_derivative_KF(start,P_bar_0,x_bar0,dt,Q,R,gam_d)
    else:
        KF_gam = error_derivative_KF(start,P_bar_0,x_bar0,dt,Q,R,gam_d,Pk_gam,xk_gam)
    Pk_gam = KF_gam[1]
    xk_gam = KF_gam[0]
    gam_dot = xk_gam[1]
#    print Pk_gam
    #integrator
    if start:
        gam_int = 0;
    gam_int += gam_d*dt

    #target angle
    if start:
        gam_target = gam_d;

    #feed-forward
    throttle_com_ff = th0/(cos(roll)*cos(pitch))

    #we may need P and I control here
    throttle_com = throttle_com_ff - KP_t*(gam_d - gam_target) - KI_t*gam_int - KD_t*gam_dot
    throttle_com = throttle_com[0]
    #throttle saturation
    #print KD_t*gam_dot
    if throttle_com > throttle_sat:
        throttle_com = throttle_sat
    elif throttle_com < 0:
        throttle_com = 0



    #SPEED CONTROL#

    #given no knowledge of drag coefficients on the aircraft take the simple
    #gain tuning approach with the concept that zero pitch gives zero speed in
    #the unperturbed case

    #forward speed from state
    #with GPS/Vicon type measurements convert this to just use the horizontal velocity in the inertial frame
    vel_f = np.dot(Rbi_image,vel)
    speed = vel_f[0]

    speed_err = speed - speed_des

    #differentiator
    #print np.power(np.array([0.01, 1]),2)
    P_bar_0 = np.diag(np.power(np.array([0.1, 1]),2))
    x_bar0 = np.concatenate([speed_err, np.array([0])]).reshape(2,1)
    #print x_bar0
    Q = np.diag(np.power(np.array([0.01, 0.1]),2))
    R = 0.1
    if start:
        KF_speed_err = error_derivative_KF(start,P_bar_0,x_bar0,dt,Q,R,speed_err)
    else:
        KF_speed_err = error_derivative_KF(start,P_bar_0,x_bar0,dt,Q,R,speed_err,Pk_speed_err,xk_speed_err)
    Pk_speed_err = KF_speed_err[1]
    xk_speed_err = KF_speed_err[0]
    speed_err_dot = xk_speed_err[1]
#    print xk_speed_err
    #integrator
    if start:
        speed_err_int = 0;
    speed_err_int += speed_err*dt




    ##differentiator
    #P_bar_0 = np.diag(np.array([0.1, 1])^2)
    #x_bar0 = np.transpose(np.array([speed_err, 0]))
    #Q = np.diag(np.array([0.01, 0.1])^2)
    #R = 0.1
    #if start:
    #	KF_speed_err = error_derivative_KF(start,P_bar_0,x_bar0,dt,Q,R,speed_err)
    #else:
    #	KF_speed_err = error_derivative_KF(start,P_bar_0,x_bar0,dt,Q,R,speed_err,Pk_speed_err,xk_speed_err)
    #Pk_speed_err = KF_speed_err[1]
    #xk_speed_err = KF_speed_err[0]
    #speed_err_dot = xk_speed_err[1]
    ##integrator
    #speed_err_int = speed_err_int + speed_err*dt

    #pitch down speeds you up
    pitch_com = KP_p*speed_err + KI_p*speed_err_int + KD_p*speed_err_dot

    #pitch saturation
    if pitch_com > pitch_sat:
        pitch_com = pitch_sat
    elif pitch_com < -pitch_sat:
        pitch_com = -pitch_sat;

    #YAW CONTROL#
    #differentiator
    #print np.power(np.array([0.01, 1]),2)
    P_bar_0 = np.diag(np.power(np.array([0.1, 1]),2))
    x_bar0 = np.concatenate([yaw_diff, np.array([0])]).reshape(2,1)
    #print x_bar0
    Q = np.diag(np.power(np.array([0.01, 0.3]),2))
    R = 1*pi/180
    if start:
        KF_yaw_diff = error_derivative_KF(start,P_bar_0,x_bar0,dt,Q,R,yaw_diff)
    else:
        KF_yaw_diff = error_derivative_KF(start,P_bar_0,x_bar0,dt,Q,R,yaw_diff,Pk_yaw_diff,xk_yaw_diff)
    Pk_yaw_diff = KF_yaw_diff[1]
    xk_yaw_diff = KF_yaw_diff[0]
    yaw_diff_dot = xk_yaw_diff[1]
#    print xk_yaw_diff
    #integrator
    if start:
        yaw_diff_int = 0;
    yaw_diff_int += yaw_diff*dt

    ##differentiator
    #P_bar_0 = np.diag(np.array([0.1, 1])^2)
    #x_bar0 = np.transpose(np.array([yaw_diff, 0]))
    #Q = np.diag(np.array([0.01, 0.3])^2)
    #R = 1*pi/180
    #if start:
    #	KF_speed_err = error_derivative_KF(start,P_bar_0,x_bar0,dt,Q,R,yaw_diff)
    #else:
    #	KF_yaw_diff = error_derivative_KF(start,P_bar_0,x_bar0,dt,Q,R,yaw_diff,Pk_yaw_diff,xk_yaw_diff)
    #Pk_yaw_diff = KF_yaw_diff[1]
    #xk_yaw_diff = KF_yaw_diff[0]
    #yaw_diff_dot = xk_yaw_diff[1]
    ##integator
    #yaw_diff_int = yaw_diff_int + yaw_diff*dt

    #command
    r_com = KP_r*yaw_diff + KI_r*yaw_diff_int + KD_r*yaw_diff_dot

    #r saturation
    if r_com > r_sat:
        r_com = r_sat
    elif r_com < -r_sat:
        r_com = -r_sat

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

    #COMMAND#
    u = [pitch_com, roll_com, r_com, throttle_com]
#    convert to commands
    data = [ 0 ] * 8
    
    if pitch_com > 0:
        RC1_cmd = -pitch_com * (RC1_ZERO - RC1_MIN)/(LIM_PITCH_MAX)*100 + RC1_ZERO
    elif pitch_cmd < 0:
        RC1_cmd = -pitch_com * (RC1_MAX - RC1_ZERO)/(-LIM_PITCH_MIN)*100 + RC1_ZERO
    else:
        RC1_cmd = RC1_ZERO
    
    if roll_com > 0:
        RC2_cmd = -roll_com * (RC2_ZERO - RC2_MIN)/(LIM_PITCH_MAX)*100 + RC2_ZERO
    elif pitch_cmd < 0:
        RC2_cmd = -roll_com * (RC2_MAX - RC2_ZERO)/(-LIM_PITCH_MIN)*100 + RC2_ZERO
    else:
        RC2_cmd = RC2_ZERO
        
    RC3_cmd = 1000 + 1000*throttle_com
    
    if r_com > 0:
        RC4_cmd = -roll_com * (RC4_ZERO - RC4_MIN)/(LIM_PITCH_MAX)*100 + RC4_ZERO
    elif pitch_cmd < 0:
        RC4_cmd = -roll_com * (RC4_MAX - RC4_ZERO)/(-LIM_PITCH_MIN)*100 + RC4_ZERO
    else:
        RC4_cmd = RC4_ZERO
    #saturation to avoid "error: ushort format requires 0 <= number <= USHRT_MAX"
    RC1_cmd = cmd_saturate(RC1_cmd,RC1_MAX,RC1_MIN)
    RC2_cmd = cmd_saturate(RC2_cmd,RC2_MAX,RC2_MIN)
    RC3_cmd = cmd_saturate(RC3_cmd,RC3_MAX,RC3_MIN)
    RC4_cmd = cmd_saturate(RC4_cmd,RC4_MAX,RC4_MIN)
    
    data[0:4] = [RC1_cmd, RC2_cmd,  RC3_cmd,  RC4_cmd]
    
    
    master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
    
    start = False





















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

