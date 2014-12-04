#import vrpn
import threading
import vrpn
import transformations
import math


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

    def tracker_callback(self, userdata,packet):
        #print packet
        fixed_axes = (packet[7], packet[4], packet[5], packet[6])
        
        #print fixed_axes
        angles = transformations.euler_from_quaternion(fixed_axes, 'rxyz')
        #print angles
        rad2deg = 180 / math.pi
        #print "CB: X: %1.4f \tY: %1.4f \tZ: %1.4f \tRoll: %1.4f \tPitch: %1.4f \tYaw: %1.4f" % \
        #     (packet[1], packet[2], packet[3], angles[1]*rad2deg, angles[0]*rad2deg, angles[2]*rad2deg)
        self.position = (packet[1], packet[2], packet[3])
        self.angles = (angles[1]*rad2deg, angles[0]*rad2deg, angles[2]*rad2deg)

    def run(self):
        t=vrpn_Tracker.vrpn_Tracker_Remote(self.hostName)
        vrpn_Tracker.register_tracker_change_handler(self.tracker_callback)
        vrpn_Tracker.vrpn_Tracker_Remote.register_change_handler(t,None,vrpn_Tracker.get_tracker_change_handler())

        while not self.stopped():
            vrpn_Tracker.vrpn_Tracker_Remote.mainloop(t)

