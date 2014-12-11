import base
import search
import target_guide
import signal
import sys
import time

global drone
drone = None

def signal_handler(signal, frame):
    global drone
    drone.stop()
    sys.exit(0)

#attach signal handler
signal.signal(signal.SIGINT, signal_handler)

#instantiate drone object
drone = base.DroneBase(vicon_name='Flamewheel', fileName = 'altitude_log.log')
drone.start()
drone.arm()

while True:
	if drone.get_area()>0:
		guide = True
	else:
		guide = False
	guide = False
	if guide:
		target_guide.target_guide(drone)
	else:
		print 'entering search'
		search.search(drone)

