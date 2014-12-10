import base
import curses

#Initalization of curses
screen = curses.initscr()
screen.clear()
screen.refresh()

drone = None
def signal_handler(signal, frame):
    global drone
    drone.stop()
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
while (1):
    curses_print("Roll RC Level: " + str(drone.current_rc_channels[0]), 1, 1)
    curses_print("Pitch RC Level: " + str(drone.current_rc_channels[1]), 2, 0)
    curses_print("Throttle RC Level: " + str(drone.current_rc_channels[2]), 3, 0)
    curses_print("Yaw RC Level: " + str(drone.current_rc_channels[3]), 4, 0)
    curses_print("Manual Control RC Level: " + str(drone.current_rc_channels[4]), 5, 0)
    time.sleep(0.5)
