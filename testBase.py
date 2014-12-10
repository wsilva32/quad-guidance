import base
import curses
import time
import signal
import sys


useCurses = 1
drone = None

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

def curses_print(string, line, col):
	"""
	Function to do a simple curses print.
	"""

	#Check for bad inputs
	if col > 1 or col < 0:
		return

	if line > 22 or line < 0:
		return

	#Print to screen using curses
	if col == 0:
		screen.addstr(line, 0, string)
	if col == 1:
		screen.addstr(line, 40, string)

	screen.refresh()



drone = base.DroneBase(vicon_name='wolverine')
drone.start()

signal.signal(signal.SIGINT, signal_handler)

time.sleep(4)
if useCurses:
    screen.clear()
    screen.refresh()

while (1):
    curLine = 0
    if useCurses:
        curses_print("Drone mode: "  + drone.mode + '   ', curLine, 0)
        curLine += 1 
        curses_print("Drone Location: %1.3f, %1.3f, %1.3f" % (drone.get_position()[0], drone.get_position()[1], drone.get_position()[2]), curLine, 0)
        curLine += 1 
        curses_print("Balloon Area: %d Centroid: (%d, %d)" % (drone.get_area(), drone.get_centroid()[0], drone.get_centroid()[1]), curLine, 0)
        curLine += 1 
        curses_print("Roll RC Level: " + str(drone.current_rc_channels[0]) + '   ',curLine, 0)
        curLine += 1 
        curses_print("Pitch RC Level: " + str(drone.current_rc_channels[1]) + '   ', curLine, 0)
        curLine += 1 
        curses_print("Throttle RC Level: " + str(drone.current_rc_channels[2]) + '   ', curLine, 0)
        curLine += 1 
        curses_print("Yaw RC Level: " + str(drone.current_rc_channels[3]) + '   ', curLine, 0)
        curLine += 1 
        curses_print("Manual Control5 RC Level: " + str(drone.current_rc_channels[4]) + '   ',curLine, 0)
        curLine += 1 
        curses_print("Manual Control6 RC Level: " + str(drone.current_rc_channels[5]) + '   ', curLine, 0)
    
    drone.set_rc_throttle(1500)
    time.sleep(0.1)
