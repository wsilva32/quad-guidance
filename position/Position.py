
import threading
import vrpn
import transformations
import math
import time

class ViconPosition(threading.Thread):
    def __init__(self, m_hostName=None):
        super(ViconPosition, self).__init__()
        self._stop = threading.Event()
        self.hostName = m_hostName
        self.position = (0,0,0)
        self.angles = (0,0,0)

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def tracker_callback(self, userdata, packet):
        #print packet
        quat = packet['quaternion']
        fixed_axes = (quat[3], quat[0], quat[1], quat[2])
        
        #print fixed_axes
        angles = transformations.euler_from_quaternion(fixed_axes, 'rxyz')
        #print angles
        rad2deg = 180 / math.pi
        
        self.position = packet['position']
        self.angles = (angles[1]*rad2deg, angles[0]*rad2deg, angles[2]*rad2deg)
        #print "CB: X: %1.4f \tY: %1.4f \tZ: %1.4f \tRoll: %1.4f \tPitch: %1.4f \tYaw: %1.4f" % \
        #     (self.position[0], self.position[1], self.position[2], self.angles[0], self.angles[1], self.angles[2])
    def run(self):
        track=vrpn.receiver.Tracker(self.hostName)

        track.register_change_handler(self,self.tracker_callback, "position", 0)

        while not self.stopped():
            track.mainloop()
            time.sleep(0.001)

