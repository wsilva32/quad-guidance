import base
import search
import target_guide
import signal
import sys
import time
import curses

global drone
drone = None
useCurses = 1

#Initalization of curses
if useCurses:
    screen = curses.initscr()

def signal_handler(signal, frame):
    global drone
    drone.stop()
    if useCurses:
        screen.clear()
        screen.refresh()
        curses.endwin()
    sys.exit(0)

#attach signal handler
signal.signal(signal.SIGINT, signal_handler)

#instantiate drone object
drone = base.DroneBase(vicon_name='Flamewheel', fileName = 'altitude_log.log')
drone.start()
#drone.arm()
time.sleep(5)

if useCurses:
    screen = curses.initscr()
    screen.clear()
    screen.refresh()

while True:
	if drone.get_area()>0:
		guide = True
	else:
		guide = False

	if guide:
		target_guide.target_guide(screen, drone)
	else:
		print 'entering search'
		search.search(drone)

