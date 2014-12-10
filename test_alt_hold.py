
useCurses = 1
drone = None

#Close throttle control around throttle
altPID = pid.PidController(100.0, 0.0, 0.0, -631, 631)

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



drone = base.DroneBase(vicon_name='Flamewheel')
drone.start()

signal.signal(signal.SIGINT, signal_handler)

time.sleep(4)
if useCurses:
    screen.clear()
    screen.refresh()

refAlt = 1.0

altPID.set(refAlt)
while (1):
    refFFThrottle = 1488/(cos(drone.get_roll())*cos(drone.get_pitch()))
    altPID.step(drone.get_position()[2])
    throt_cmd = refFFThrottle + altPID.get()
    drone.set_rc_throttle(throt_cmd)

    if useCurses:
        curLine = 0
        curses_print("Drone mode: "  + drone.mode + '   ', curLine, 0)
        curLine += 1 
        curses_print("Altitude: %1.3f Error: %1.3f" % (drone.get_position()[2], refAlt - drone.get_position()[2]), curLine,0)
        curLine += 1
        curses_print("Throttle Command: %d" % throt_cmd , curLine,0)
        curLine += 1
    time.sleep(0.1)
