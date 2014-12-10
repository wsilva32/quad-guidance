
from pymavlink import mavutil
import sys, math, time
import socket, struct, threading
import logging
from position import Position
import camera.BalloonFinder

class DroneBase(threading.Thread):

	def __init__(self, vicon_name='Flamewheel', image_width=320, image_height=240):
            super(DroneBase, self).__init__()
            self._stop = threading.Event()
            #for GCS with wireless radio
            #self.baud = 57600 
            #self.device = '/dev/ttyUSB0'
            
            #for raspberry pi
            self.baud = 115200 
            self.device = '/dev/ttyACM0'
            
            self.base_rc_roll = 1519
            self.base_rc_pitch = 1519
            self.base_rc_throttle = 1516
            self.base_rc_yaw = 1520

            #Basic runtime mode
            self.mode = 'MANUAL'

            #Home x,y,x position
            self.home_x = 0
            self.home_y = 0
            self.home_z = 0

            #Last updated position
            self.position = (0.0,0.0,0.0)

            #Last updated rc channel's values'
            self.current_rc_channels = [None] * 6
            
            #Last updated rc overrides
            self.current_rc_overrides = [0] * 6
            
            #Last updated attitude
            self.current_pitch = None
            self.current_yaw = None
            self.current_roll = None
            
            #Fence for safety (Not implemented yet)
            self.fence_x_min = None
            self.fence_x_max = None
            self.fence_y_min = None
            self.fence_y_max = None
            self.fence_z_min = None
            self.fence_z_max = None
                
            self.clock = time.time()

            self.vicon = Position.ViconPosition(vicon_name + "@192.168.20.10")
            self.vicon.start()
            
            self.cam = camera.BalloonFinder.BalloonFinder(320,240)
            self.cam.start()

            #Start of a log
            logging.basicConfig(filename='vidro.log', level=logging.DEBUG)

        def stop(self):
            self.master.arducopter_disarm()
            self.cam.stop()
            self.vicon.stop()
            self._stop.set()

        def stopped(self):
            return self._stop.isSet()
            
        def run(self):
            self.connect_mavlink()
            while not self.stopped():
                self.update_mavlink()
                time.sleep(0.01)
                
	def connect_mavlink(self):
		"""
		Initialize connection to pixhawk and make sure to get first heartbeat message
		"""
		#Initialize connection
		self.master = mavutil.mavlink_connection(self.device, self.baud)
		print "Attempting to get HEARTBEAT message from APM..."

		#Request heartbeat from APM
		msg = self.master.recv_match(type='HEARTBEAT', blocking=True)
		print("Heartbeat from APM (system %u component %u)" % (self.master.target_system, self.master.target_system))
                
		#The max rate (the second to last argument in the line below) is 25 Hz. You must change the firmware to get a fast rate than that.
		#It may be possible to get up to 500 Hz??
		#This may be useful later down the road to decrease latency
		#It also may be helpful to only stream needed data instead of all data

                #setup data stream
                self.master.mav.request_data_stream_send(self.master.target_system, self.master.target_component, 0, 1, 0) #All
                self.master.mav.request_data_stream_send(self.master.target_system, self.master.target_component, 3, 25, 1) #RC channels
                #self.master.mav.request_data_stream_send(self.master.target_system, self.master.target_component, 6, 25, 1) #Position
			
                #Get intial values from the APM
                print "Getting inital values from APM..."
                while (self.current_rc_channels[0] == None):
                    self.update_mavlink()
                    print("Got RC channels")
                
                #set mode to stabilize
                self.master.set_mode('STABILIZE')

                #Set arming check to 0
                self.master.param_set_send('ARMING_CHECK',0)
                self.rc_all_reset()
                self.send_rc_overrides()
                self.mode = 'MANUAL'     

                #disable DCM Threshold check
                self.master.param_set_send('DCM_CHECK_THRESH',0)
                #http://copter.ardupilot.com/wiki/ekf-inav-failsafe/

        def arm(self):
                #Arm ArduCopter
                self.master.arducopter_arm()
                time.sleep(5)	#delay to wait for arm to complete

	def update_mavlink(self):
		"""
		Function for getting the general mavlink message and changing class variables based on that message.
		It looks for 5 different message types:
		-'BAD_DATA'
		-'RC_CHANNELS_RAW'
		-'GLOBAL_POSITION_INT' (for SITL only)
		-'ATTITUDE'
		-'HEARTBEAT'
		This is non-blocking so does not gaurantee to chnage any values.
		"""
		self.msg = self.master.recv_match(blocking=True)

		if self.msg:
			#print self.msg.get_type()

                    #manual override: Channel 5
			if self.msg.get_type() == "RC_CHANNELS_RAW":
				try:
					self.current_rc_channels[0] = self.msg.chan1_raw
					self.current_rc_channels[1] = self.msg.chan2_raw
					self.current_rc_channels[2] = self.msg.chan3_raw
					self.current_rc_channels[3] = self.msg.chan4_raw
					self.current_rc_channels[4] = self.msg.chan5_raw
					self.current_rc_channels[5] = self.msg.chan6_raw
				except:
					pass
                                        
				#Implement a safety if the manual control is override is set
                                if self.current_rc_channels[4] > 1500:
                                    #Manual mode
                                    #Release controls
                                    self.rc_all_reset()
                                    self.send_rc_overrides()
                                    self.mode = 'MANUAL'
                                else:
                                    self.mode = 'STAB'
                                    self.send_rc_overrides()

                                #print 'RC:', self.current_rc_channels

	def close(self):
		"""
		Call at the end of all programs.
		"""
                self.vicon.stop()
                self.cam.stop()
                
	def get_position(self):
            return self.vicon.position

	def set_fence(min_x, max_x, min_y, max_y, min_z, max_z):
		"""
		Set the fence for the quadcopter to stay within. This currently just sets global fence variables.
		"""
		self.fence_x_min = mix_x
		self.fence_x_max = max_x
		self.fence_y_min = min_y
		self.fence_y_max = max_y
		self.fence_z_min = min_z
		self.fence_z_max = max_z

	def rc_filter(self, rc_value, rc_min, rc_max):
		"""
		Filter for the RC values to filter out RC values out of RC range. Returns filtered RC value
		"""
		if rc_value > rc_max:
			rc_value = rc_max
		if rc_value < rc_min:
			rc_value = rc_min
		return rc_value

	def send_rc_overrides(self):
            if self.mode == 'STAB':
		self.master.mav.rc_channels_override_send(self.master.target_system, self.master.target_component, self.current_rc_overrides[0], self.current_rc_overrides[1], self.current_rc_overrides[2], self.current_rc_overrides[3], self.current_rc_overrides[4], self.current_rc_overrides[5], 0, 0)
		#self.master.mav.file.fd.flush()

	## Set RC Channels ##
	def set_rc_roll(self, rc_value):
		rc_value = self.rc_filter(rc_value,1519-270,1519+270)
		self.current_rc_overrides[0] = rc_value

	def set_rc_pitch(self, rc_value):
		rc_value = self.rc_filter(rc_value, 1519-270,1519+270)
		self.current_rc_overrides[1] = rc_value


	def set_rc_throttle(self, rc_value):
		rc_value = self.rc_filter(rc_value, 1110, 1741)
		self.current_rc_overrides[2] =  rc_value


	def set_rc_yaw(self, rc_value):
		rc_value = self.rc_filter(rc_value, 1277, 1931)
		self.current_rc_overrides[3] = rc_value



	## Reset RC Channels ##
	def rc_roll_reset(self):
		self.current_rc_overrides[0] = 0


	def rc_pitch_reset(self):
		self.current_rc_overrides[1] = 0


	def rc_throttle_reset(self):
		self.current_rc_overrides[2] = 0


	def rc_yaw_reset(self):
		self.current_rc_overrides[3] = 0


	def rc_channel_five_reset(self):
		self.current_rc_overrides[4] = 0


	def rc_channel_six_reset(self):
		self.current_rc_overrides[5] = 0

	def rc_all_reset(self):
		self.rc_roll_reset()
		self.rc_pitch_reset()
		self.rc_throttle_reset()
		self.rc_yaw_reset()

	def rc_check_dup(self, channel, value):
		"""
		Check for duplicate RC value. This is used to check to see if the RC value is already set to the value being passed in.
		"""
		if self.v.channel_readback[channel] == value:
			return True
		return False

	def get_roll(self):
            """
            Returns the roll in radians
            """
            return self.vicon.angles[0]

        def get_pitch(self):
            """
            Returns the pitch of the copter in radians
            """
            return self.vicon.angles[1]
        
        def get_yaw(self):
            return self.vicon.angles[2]

        def get_area(self):
            return self.cam.area

        def get_centroid(self):
            return self.cam.centroid
            
        def get_framerate(self):
            return self.cam.frameRate

        def get_vidsize(self):
            return self.cam.vidSize

        def get_fov(self):
            return (self.cam.fovH, self.cam.fovV)

