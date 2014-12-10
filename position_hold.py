import base
import pid
from math import *

class AltitudeHold(object):
    def __init__(self, m_drone ):
        self.drone = m_drone
        self.ref = 0.0
        self.altPID = pid.PidController(100.0, 0.0, 0.0, -631, 631)

    def setRef(self, m_ref):
        self.altPID.set(m_ref)
        self.ref = m_ref

    def getThrottle(self):
        refFFThrottle = 1488/(cos(self.drone.get_roll())*cos(self.drone.get_pitch()))
        self.altPID.step(self.drone.get_position()[2])
        throt_cmd = refFFThrottle + altPID.get()
        return throt_cmd
        
    def getError(self):
        return (self.ref - self.drone.get_position()[2])

