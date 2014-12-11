import base
import pid
from math import *

class AltitudeHold(object):
	def __init__(self, m_drone ):
		self.baseThrottle = 1525
		self.drone = m_drone
		self.ref = 0.0
		self.altPID = pid.PidController(0.0, 0.0, -50.0, -631, 631) #PID, low bound, up bound

	def setRef(self, m_ref):
		self.altPID.set(m_ref)
		self.ref = m_ref

	def getThrottle(self):
		refFFThrottle = self.baseThrottle/(cos(self.drone.get_roll())*cos(self.drone.get_pitch()))
		self.altPID.step(self.drone.get_position()[2])
		throt_cmd = refFFThrottle + self.altPID.get()
		return throt_cmd

	def getError(self):
		return (self.ref - self.drone.get_position()[2])

class LateralHold(object):
	def __init__(self, m_drone ):
		self.drone = m_drone
		self.x_ref = 0.0
		self.y_ref = 0.0
		self.yaw = 0.
		self.latPID = pid.PidController(0.0, 0.0, -50.0, -631, 631) #PID, low bound, up bound gains positive
		self.lonPID = pid.PidController(0.0, 0.0, -50.0, -631, 631) #PID, low bound, up bound gains negative

	def setRef(self, x_ref, y_ref):
		self.latPID.set(0)
		self.lonPID.set(0)
		self.x_ref = x_ref
		self.y_ref = y_ref

	def getRoll(self):
		self.yaw = self.drone.get_yaw()
		lat = -(self.x_ref-self.drone.get_position()[0])*sin(self.yaw) + (self.y_ref-self.drone.get_position()[1])*cos(self.yaw)
		self.latPID.step(lat)
		roll_cmd = self.latPID.get()
		return roll_cmd

	def getPitch(self):
		self.yaw = self.drone.get_yaw()
		lon = (self.x_ref-self.drone.get_position()[0])*cos(self.yaw) + (self.y_ref-self.drone.get_position()[1])*sin(self.yaw)
		self.lonPID.step(lon)
		pitch_cmd = self.lonPID.get()
		return pitch_cmd

	def getError(self):
		return (self.x_ref - self.drone.get_position()[0]), (self.y_ref - self.drone.get_position()[1])

