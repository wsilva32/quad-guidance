def target_guide(drone):
	#Imports
	#import sys, struct, time, os, signal
	#from pymavlink import mavutil
	#import numpy as np
	#from math import *
	import signal
	from attitude_tools import angle2dcm
	from KF_error import error_derivative_KF
	from velocity_KF import velocity_KF
	#from coordtrans import lla2flatdumb
	#from cmd_saturate import cmd_saturate
	#import camera.BalloonFinder
	#import FakePosition
	#from Position import Position
	#import transformations
	import Base
	Drone = Drone.Base

	#from target_sim import target_sim

	#Constants
	#vicon_object = "Flamewheel"
	vicon_object = "wolverine"

	runtimeMode = 'debug'
	#runtimeMode = 'run'

	#Define functions
	def signal_handler(signal, frame):
	    global v
	    print('Exiting controller')
	    v.stop()
	    cam.stop()
	    sys.exit(0)


	#Program Start

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
	g = 9.807
	m = 2
	#level hover thrust fraction
	th0 = 0.6

	#field of view from center in degrees
	fov = drone.get_fov()

	FoVh = fov[0]
	FoVv = fov[1]

	#field of view in pixels from center
	vidsize = drone.get_vidsize()
	FoVph = vidsize[0]
	FoVpv = vidsize[1]

	#range to virtual plane in pixels
	rangeh = FoVph/tan(FoVh)
	rangev = FoVpv/tan(FoVv)
	start = True

	#initialize
	prev = time.time()
	gam_target = 0;

	#Velocity KF inputs
	#could all be moved ahead of the loop
	pos_vic = np.array(drone.get_position())
	R_vic = np.identity(3)*0.001**2
	Q_vic = np.identity(6)*0.1**2
	P_bar_0_vic = np.identity(6)
	x_bar0_vic = np.concatenate([pos_vic, np.array([0,0,0])])
	x_bar0_vic = x_bar0_vic.reshape((6, 1))

	RC1_MAX = 1925.000000
	RC1_MIN =  1106.000000
	#RC1_ZERO = 1500

	RC2_MAX = 1930.000000
	RC2_MIN = 1110.000000
	#RC2_ZERO = 1500

	RC3_MAX = 1929.000000
	RC3_MIN = 1110.000000
	#RC3_ZERO = 1500

	RC4_MAX = 1933.000000
	RC4_MIN = 1109.000000
	#RC4_ZERO = 1500

	LIM_PITCH_MIN = -4500.00000/100 * pi/180
	LIM_PITCH_MAX = 4500.00000/100 * pi/180

	LIM_ROLL = 4500.00000/100 * pi/180

	LIM_YAW_RATE = 4.500000*200/4.5 * pi/180

	#Alt KF inputs
	while True:
	    #check target status
	    if drone.get_area()==0:
	        break

	    loopStart = getMilliTime()

	    #get the time step
	    dt = time.time() - prev
	    prev = time.time()
	    #Get velocity (vel), roll and pitch (roll and pitch from aircraft, velocity from filtering vicon)
	    roll = drone.get_roll()
	    pitch = drone.get_pitch()
	    yaw = drone.get_yaw() #verify what order angles is in

	    #position from vicon (invert y and z to flip into z down coordinate frame)
	    pos_vic = np.array(drone.get_position())
	    pos_vic[1] = -pos_vic[1]
	    pos_vic[2] = -pos_vic[2]


	    target = drone.get_centroid
	    acquireTime = getMilliTime()

	    #estimate the velocity (would like to move this to a paralell operation at higher rate, will need dt calculated in that loop as well)
	    if start:
		vkf = velocity_KF(start,P_bar_0_vic,x_bar0_vic,dt,Q_vic,R_vic,pos_vic.reshape((3,1)))
	    else:
		vkf = velocity_KF(start,P_bar_0_vic,x_bar0_vic,dt,Q_vic,R_vic,pos_vic.reshape((3,1)),Pk_vic,x_vic)

	    Pk_vic = vkf[1]
	    x_vic = vkf[0]
	    vel_i = x_vic[3:6]

	    #rotate inertial frame velocity to aircraft frame
	    Rib = angle2dcm(yaw,roll,pitch)
	    vel = np.dot(Rib,vel_i)
	    print 'Vel: %f %f %f' % (vel[0], vel[1], vel[2])

	    velTime = getMilliTime()

	    #MEASUREMENT CONVERSION# (target pixels to desired flight path and yaw difference)

	    #z down
	    
	    az = atan2(target[0], rangeh)
	    el = atan2(-target[1], rangev)

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

	    measureTime = getMilliTime()
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
	    if start:
		#print np.power(np.array([0.01, 1]),2)
		P_bar_0_alt = np.diag(np.power(np.array([0.01, 1]),2))
		x_bar0_alt = np.concatenate([gam_d, np.array([0])]).reshape(2,1)
		#print x_bar0
		Q_alt = np.diag(np.power(np.array([0.01, 0.3]),2))
		R_alt = 1*pi/180
	    if start:
		KF_gam = error_derivative_KF(start,P_bar_0_alt,x_bar0_alt,dt,Q_alt,R_alt,gam_d)
	    else:
		KF_gam = error_derivative_KF(start,P_bar_0_alt,x_bar0_alt,dt,Q_alt,R_alt,gam_d,Pk_gam,xk_gam)
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
	    #print gam_d
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
	    altconTime = getMilliTime()


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
	    if start:
		P_bar_0_speed = np.diag(np.power(np.array([0.1, 1]),2))
		x_bar0_speed = np.concatenate([speed_err, np.array([0])]).reshape(2,1)
		Q_speed = np.diag(np.power(np.array([0.01, 0.1]),2))
		R_speed = 0.1
	    if start:
		KF_speed_err = error_derivative_KF(start,P_bar_0_speed,x_bar0_speed,dt,Q_speed,R_speed,speed_err)
	    else:
		KF_speed_err = error_derivative_KF(start,P_bar_0_speed,x_bar0_speed,dt,Q_speed,R_speed,speed_err,Pk_speed_err,xk_speed_err)
	    Pk_speed_err = KF_speed_err[1]
	    xk_speed_err = KF_speed_err[0]
	    speed_err_dot = xk_speed_err[1]
	    #    print xk_speed_err
	    #integrator
	    if start:
		speed_err_int = 0;
	    elif KI_p*speed_err_int > pitch_sat:
		speed_err_int = speed_err_int
	    else:
		speed_err_int += speed_err*dt
	#    clear integrator
	    if KP_p*speed_err>pitch_sat or speed_err<0.05:
		speed_err_int = 0;


	    #pitch down speeds you up
	    pitch_com = KP_p*speed_err + KI_p*speed_err_int + KD_p*speed_err_dot

	    #pitch saturation
	    if pitch_com > pitch_sat:
		pitch_com = pitch_sat
	    elif pitch_com < -pitch_sat:
		pitch_com = -pitch_sat;
	    speedconTime = getMilliTime()
	    #YAW CONTROL#
	    #differentiator
	    #print np.power(np.array([0.01, 1]),2)
	    #P_bar_0_yaw = np.diag(np.power(np.array([0.1, 1]),2))
	    #x_bar0_yaw = np.concatenate([yaw_diff, np.array([0])]).reshape(2,1)
	    #print x_bar0
	    #Q_yaw = np.diag(np.power(np.array([0.01, 0.3]),2))
	    #R_yaw = 1*pi/180
	    #if start:
	    #		KF_yaw_diff = error_derivative_KF(start,P_bar_0_yaw,x_bar0_yaw,dt,Q_yaw,R_yaw,yaw_diff)
	    #	else:
	    #		KF_yaw_diff = error_derivative_KF(start,P_bar_0_yaw,x_bar0_yaw,dt,Q_yaw,R_yaw,yaw_diff,Pk_yaw_diff,xk_yaw_diff)
	    #	Pk_yaw_diff = KF_yaw_diff[1]
	    #	xk_yaw_diff = KF_yaw_diff[0]
	    #	yaw_diff_dot = xk_yaw_diff[1]
	    #    print xk_yaw_diff
	    #integrator
	    if start:
		yaw_diff_int = 0;
	    yaw_diff_int += yaw_diff*dt

	    #command
	    yaw_diff_dot = 0
	    r_com = KP_r*yaw_diff + KI_r*yaw_diff_int + KD_r*yaw_diff_dot

	    #r saturation
	    if r_com > r_sat:
		r_com = r_sat
	    elif r_com < -r_sat:
		r_com = -r_sat
	    yawconTime = getMilliTime()
	    #ROLL CONTROL#

	    #sideways motion
	    if not start:
		sw_slip_prev = sw_slip
	    sw_slip = vel[2]


	    #differentiator
	    #print np.power(np.array([0.01, 1]),2)
	    if start:
		P_bar_0_roll = np.diag(np.power(np.array([0.1, 1]),2))
		x_bar0_roll = np.concatenate([sw_slip, np.array([0])]).reshape(2,1)
		Q_roll = np.diag(np.power(np.array([0.01, 0.3]),2))
		R_roll = 0.1
	    if start:
		KF_sw_slip = error_derivative_KF(start,P_bar_0_roll,x_bar0_roll,dt,Q_roll,R_roll,sw_slip)
	    else:
		KF_sw_slip = error_derivative_KF(start,P_bar_0_roll,x_bar0_roll,dt,Q_roll,R_roll,sw_slip,Pk_sw_slip,xk_sw_slip)
	    Pk_sw_slip = KF_sw_slip[1]
	    xk_sw_slip = KF_sw_slip[0]
	    sw_slip_dot = xk_sw_slip[1]

	    #integrator
	    if start:
		sw_slip_int = 0;
	    sw_slip_int += sw_slip*dt

	    #command (positive roll -> positive slip)
	    roll_com = -KP_rl*sw_slip -KI_rl*sw_slip_int -KD_rl*sw_slip_dot

	    #roll saturation
	    if roll_com > roll_sat:
		roll_com = roll_sat
	    elif roll_com < -roll_sat:
		roll_com = -roll_sat
	    rollconTime = getMilliTime()
	    #COMMAND#
	    u = [pitch_com, roll_com, -r_com, throttle_com]

	    if runtimeMode == 'debug':
		print "Pitch: %f Roll: %f YawRate: %f Throttle: %f" % (u[0], u[1], u[2], u[3])
	#        print 'Cam framerate: %f' % cam.frameRate

		print 'Mav Time: %f' % (mavlinkTime - loopStart)
		print 'Acq time: %f' % (acquireTime - mavlinkTime) 
		print 'Vel time: %f' % (velTime - acquireTime)
		print 'Measurement Conversion time: %f' % (measureTime - velTime)
		print 'Alt Control time: %f' % (altconTime - measureTime)
		print 'Speed Control time: %f' % (speedconTime - altconTime)
		print 'Yaw Control time: %f' % (yawconTime - speedconTime)
		print 'Roll Control time: %f' % (rollconTime - yawconTime)
		print 'Total Control time: %f' % (getMilliTime() - velTime)
	    #    convert to commands
	    data = [ 0 ] * 8

	    RC2_cmd = RC2_MIN + (RC2_MAX - RC2_MIN)*(pitch_com+LIM_PITCH/(2*LIM_PITCH))

	    #if pitch_com > 0:
		#	RC2_cmd = -pitch_com * (RC2_ZERO - RC2_MIN)/(LIM_PITCH_MAX) + RC2_ZERO
	    #elif pitch_com < 0:
		#	RC2_cmd = pitch_com * (RC2_MAX - RC2_ZERO)/(LIM_PITCH_MIN) + RC2_ZERO
	    #else:
		#	RC2_cmd = RC2_ZERO

	    RC1_cmd = RC1_MIN + (RC1_MAX - RC1_MIN)*(roll_com+LIM_ROLL/(2*LIM_ROLL))

	    #if roll_com > 0:
		#	RC1_cmd = roll_com * (RC1_ZERO - RC1_MIN)/(LIM_ROLL) + RC1_ZERO
	    #elif roll_com < 0:
		#	RC1_cmd = roll_com * (RC1_MAX - RC1_ZERO)/(LIM_ROLL) + RC1_ZERO
	    #else:
		#	RC1_cmd = RC1_ZERO
		
	    RC3_cmd = RC3_MIN + (RC3_MAX - RC3_MIN)*throttle_com

	    RC4_cmd = RC4_MIN + (RC4_MAX - RC4_MIN)*(r_com+LIM_YAW_RATE/(2*LIM_YAW_RATE))

	    #if r_com > 0:
		#	RC4_cmd = r_com * (RC4_ZERO - RC4_MIN)/(LIM_YAW_RATE) + RC4_ZERO
	    #elif r_com < 0:
		#	RC4_cmd = r_com * (RC4_MAX - RC4_ZERO)/(LIM_YAW_RATE) + RC4_ZERO
	    #else:
		#	RC4_cmd = RC4_ZERO
	    # set RC values
	    drone.set_rc_roll(RC2_cmd)
	    drone.set_rc_pitch(RC1_cmd)
	    drone.set_rc_yaw(RC4_cmd)
	    drone.set_rc_throttle(RC3_cmd)

		#Human instruction
		if roll_com > 0:
			print 'ROLL LEFT'
		elif roll_com < 0:
			print 'ROLL RIGHT'

		if pitch_com < 0:
			print 'PITCH BACK'
		elif  pitch_com > 0:
			print 'PITCH FORWARD'

		if r_com > 0:
			print 'YAW COUNTER-CLOCKWISE'
		elif rcom < 0:
			print 'YAW CLOCKWISE'

		print 'THROTTLE: %f\%' % throttle_com*100

	    start = False

