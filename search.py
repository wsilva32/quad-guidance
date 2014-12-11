import position_hold
import time
from cmd_saturate import cmd_saturate

from numpy import *

def search(drone):
	alt_cmd = 1.0 #meter, floor = 0.0, positive up
	x_cmd = 1.0 #meter
	y_cmd = 1.0 #meter

	alt = position_hold.AltitudeHold(drone)
	alt.setRef(alt_cmd)

	horz = position_hold.LateralHold(drone)
	horz.setRef(x_cmd,y_cmd)

	RC1_MAX = 1925.000000
	RC1_MIN =  1106.000000

	RC2_MAX = 1930.000000
	RC2_MIN = 1110.000000

	RC3_MAX = 1929.000000
	RC3_MIN = 1110.000000

	RC4_MAX = 1933.000000
	RC4_MIN = 1109.000000

	LIM_ROLL = 4500.00000/100 * pi/180
	LIM_PITCH = 4500.00000/100 * pi/180
	LIM_YAW_RATE = 4.500000*200/4.5 * pi/180

        drone.writeHeader('roll\tpitch\tyaw\tpositionZ\tthrottle\terror\toverride')
	while True:
		#get controller error and pitch/roll commands
		xerror,yerror = horz.getError()
		RC1_cmd = horz.getPitch()
		RC2_cmd = horz.getRoll()

		#get controller error and throttle command
		alterror = alt.getError() #returns floating error (m)
		RC3_cmd = alt.getThrottle() #returns PWM cmd

		drone.log('%1.4f\t%1.4f\t%1.4f\t%1.4f\t%d\t%1.4f\t%d' % 
                          (drone.get_roll(), drone.get_pitch(), drone.get_yaw(), 
                           drone.get_position()[2], RC3_cmd,alterror, drone.current_rc_channels[4]))
		
		#set soft limits
		softroll = 10 * pi/180 #10 degrees
		soft_RC1_max = RC1_MIN + (RC1_MAX - RC1_MIN)*(softroll+LIM_ROLL/(2*LIM_ROLL))
		soft_RC1_min = RC1_MIN + (RC1_MAX - RC1_MIN)*(-softroll+LIM_ROLL/(2*LIM_ROLL))
		
		softpitch = 10 * pi/180 #10 degrees
		soft_RC2_max = RC2_MIN + (RC2_MAX - RC2_MIN)*(softpitch+LIM_PITCH/(2*LIM_PITCH))
		soft_RC2_min = RC2_MIN + (RC2_MAX - RC2_MIN)*(-softpitch+LIM_PITCH/(2*LIM_PITCH))

		#issue RC overrides from controllers
		#drone.set_rc_roll(cmd_saturate(RC1_cmd,soft_RC1_max,soft_RC1_min))
		#drone.set_rc_pitch(cmd_saturate(RC2_cmd,soft_RC2_max,soft_RC2_min))
		drone.set_rc_throttle(RC3_cmd)

		#Check if altitude is reached
		if sqrt(alterror**2 + xerror**2 + yerror**2) <= 0.1:
			r_com = 10*pi/180 #10 deg/s yaw rate command
			LIM_YAW_RATE = 4.500000*200/4.5 * pi/180
			RC4_cmd = RC4_MIN + (RC4_MAX - RC4_MIN)*(r_com+LIM_YAW_RATE/(2*LIM_YAW_RATE))
			#drone.set_rc_yaw(RC4_cmd)
			print 'yawing: %d ' % RC4_cmd
		if drone.get_area() > 0:
			break

        time.sleep(0.05)
		
	
