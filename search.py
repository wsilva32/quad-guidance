import position_hold
import time

from numpy import *

def search(drone):
	alt_cmd = 1.5 #meter
	alt = position_hold.AltitudeHold(drone)
	alt.setRef(alt_cmd)
	while True:
		alterror = alt.getError() #returns floating error (m)
		RC3_cmd = alt.getThrottle() #returns PWM cmd

		drone.log('positionZ:\t%1.4f\tthrottle:\t%d\terror\t%1.4f\t%d' % (drone.get_position()[2], RC3_cmd,alterror, drone.current_rc_channels[4]))

		drone.set_rc_throttle(RC3_cmd)
		if abs(alterror) <= 0.1:
			r_com = 10*pi/180 #10 deg/s yaw command
			RC4_MAX = 1933.000000
			RC4_MIN = 1109.000000
			RC4_ZERO = 1500
			LIM_YAW_RATE = 4.500000*200/4.5 * pi/180
			if r_com > 0:
				RC4_cmd = r_com * (RC4_ZERO - RC4_MIN)/(LIM_YAW_RATE) + RC4_ZERO
			elif r_com < 0:
				RC4_cmd = r_com * (RC4_MAX - RC4_ZERO)/(LIM_YAW_RATE) + RC4_ZERO
			else:
				RC4_cmd = RC4_ZERO
			drone.set_rc_yaw(RC4_cmd)
			print 'yawing: %d ' % RC4_cmd
		if drone.get_area() > 0:
			break

                time.sleep(0.05)
		
	
