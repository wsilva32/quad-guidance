
from pymavlink import mavutil
import sys, math, time
import socket, struct, threading
import logging
from position import Position

class DroneBase:

	def __init__(self, vicon_name='Flamewheel'):

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

            self.vicon = Position.ViconPosition("l@192.168.20.10")

            #Start of a log
            logging.basicConfig(filename='vidro.log', level=logging.DEBUG)

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
                self.master.mav.request_data_stream_send(self.master.target_system, self.master.target_component, 6, 25, 1) #Position
			
                #Get intial values from the APM
                print "Getting inital values from APM..."
                while (self.current_rc_channels[0] == None):
                    self.update_mavlink()
                    print("Got RC channels")
                    
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
		self.msg = self.master.recv_match(blocking=False)

		if self.msg:
			#print self.msg.get_type()

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
				
				if self.vicon_time >= self.get_vicon()[0]:
					logging.error('Vicon system values are remainng the same. Stop the system and restart the vicon values')
					self.set_rc_throttle(self.base_rc_throttle)
					self.set_rc_roll(self.base_rc_roll)
					self.set_rc_pitch(self.base_rc_pitch)
					self.set_rc_yaw(self.base_rc_yaw)
					self.vicon_error = True
				self.vicon_time = self.get_vicon()[0]

				self.send_rc_overrides()

			if self.sitl == True:
				if self.msg.get_type() == "ATTITUDE":
					self.current_yaw = self.msg.yaw*180/math.pi

				if self.msg.get_type() == "GLOBAL_POSITION_INT":
					self.current_lat = self.msg.lat * 1.0e-7
					self.current_lon = self.msg.lon * 1.0e-7
					self.current_alt = self.msg.alt-self.ground_alt

	def connect_vicon(self):
		"""
		Connects to vicon. This is needed to scream vicon data.
		"""
		self.s = ViconStreamer()
		self.s.connect("Vicon", 800)
		self.streams = self.s.selectStreams(["Time", "t-", "a-"])
		self.s.startStreams(verbose=False)
		print "checking values..."
		while self.s.getData() == None:
			pass
			
		print "Got inital vicon position"
		self.home_x = self.get_position()[0]
		self.home_y = self.get_position()[1]
		self.home_z = self.get_position()[2]
		
		timer = time.time()
		self.vicon_time = self.get_vicon()[0]
		time.sleep(1)
		while self.vicon_time >= self.get_vicon()[0]:
			if (time.time() - timer) > 10:
				print "unable to connect to the vicon system. Needs to be reset"
				logging.error('Unable to connect to the vicon system. Needs to be reset')
				return False
		print "Vicon Connected..."
		return True
		"""
		if len(self.s.getData()) < 51:
			self.num_vicon_objs = 1
		elif len(self.s.getData()) > 50:
			self.num_vicon_objs = 2
		else:
			logging.error('Number of Vicon objects was not set. This means that length of s.getData() was not correct')
		"""

	def disconnect_vicon(self):
		"""
		Properly closes vicon connection. Call this when finished using vicon.
		"""
		self.s.close()

	def connect(self):
		"""
		Connects to mavlink and vicon
		"""
		flight_ready = True
	
                flight_ready = self.connect_vicon()
		self.connect_mavlink()
		return flight_ready

	def close(self):
		"""
		Call at the end of all programs.
		"""
		if self.sitl == False:
			self.disconnect_vicon()

	def get_vicon(self):
		"""
		Gets vicon data in the folling format:

		if num_vicon_objs == 1:
			vicon_data()[0] = time
			vicon_data()[1] = x
			vicon_data()[2] = y
			vicon_data()[3] = z
			vicon_data()[4] = x rotation
			vicon_data()[5] = y rotation
			vicon_data()[6] = z rotation

		if num_vicon_objs == 2:
			vicon_data()[0] = time
			vicon_data()[1] = x_1
			vicon_data()[2] = y_1
			vicon_data()[3] = z_1
			vicon_data()[4] = x_2
			vicon_data()[5] = y_2
			vicon_data()[6] = z_2
			vicon_data()[7] = x_rotation_1
			vicon_data()[8] = y_rotation_1
			vicon_data()[9] = z_rotation_1 (yaw)
			vicon_data()[10] = x_rotation_2
			vicon_data()[11] = y_rotation_2
			vicon_data()[12] = z_rotation_2
		"""
		return self.s.getData()

	def set_vicon_home(self):
		"""
		Set the home coordinate for the vicon data.
		"""
		self.home_x = self.get_vicon()[1]
		self.home_y = self.get_vicon()[2]
		self.home_z = self.get_vicon()[3]

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

	def get_alt(self):
		"""
		Returns the altitude in mm in SITL
		"""
		return self.current_alt

	def get_lat(self):
		"""
		Returns the latitude of the copter in SITL
		"""
		return self.current_lat

	def get_lon(self):
		"""
		Returns the longitude of the copter in SITL
		"""
		return self.current_lon

	def get_roll(self):
		"""
		Returns the roll in radians
		"""
		return self.current_roll

	def get_yaw_radians(self):
		"""
		Returns the current yaw in radians from -pi to pi
		Works for both SITL and Vicon
		For SITL it returns that yaw givn by the copter and for the Vicon system it returns the yaw given by the Vicon
		"""
		yaw = None

		if self.sitl == True:
			yaw = self.current_yaw

		else:
			try:
				#Depending on different number of objects yaw located in different place in the vicon data stream
				if self.num_vicon_objs == 1:
					yaw = self.get_vicon()[6]*(1.0)
				if self.num_vicon_objs == 2:
					yaw = self.get_vicon()[9]*(1.0)
				self.vicon_error = False
			except:
				logging.error('Unable to get the yaw(radians) from vicon')
				yaw = None
				self.vicon_error = True
		return yaw

	def get_yaw_degrees(self):
		"""
		Returns the current yaw in degrees from 0 to 360
		Works for SITL and Vicon
		For SITL it returns that yaw givn by the copter and for the Vicon system it returns the yaw given by the Vicon
		"""
		try:
			if self.num_vicon_objs == 1:
				yaw = math.degrees((self.get_vicon()[6]*(1.0)) % ((2*math.pi)*(1.0)))*-1
			if self.num_vicon_objs == 2:
				yaw = math.degrees((self.get_vicon()[9]*(1.0)) % ((2*math.pi)*(1.0)))*-1
			self.vicon_error = False
			if yaw < 0.0:
				yaw += 360

		except:
			logging.error('Unable to get the yaw(radians) from vicon')
			yaw = None
			self.vicon_error = True

		return yaw

	def get_pitch(self):
		"""
		Returns the pitch of the copter in radians
		"""
		return self.current_pitch

	def get_position(self):
		"""
		Will return position in millimeters. (X,Y,Z)
		Use this for Vicon and SITL in the loop.
		"""
		position=[None]*3

                
                try:
                    position[0] = self.get_vicon()[1]
                position[1] = self.get_vicon()[2]
                position[2] = self.get_vicon()[3]
				self.vicon_error = False
			except:
				logging.error('Unable to get position data from vicon')
				position = None
				self.vicon_error = True
		return position

	def get_distance_xy(self):
		"""
		Returns the distance traveled from home in millimeters
		"""
                
                distance = math.sqrt(self.get_position()[0]*self.getposition()[0] + self.get_position()[1]*self.get_position()[1])
		return distance

	def set_home(self):
		"""
		Sets the home for he quadcopter.
		Use this for Vicon 
                """
                self.set_vicon_home()


