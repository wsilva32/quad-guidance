from position import Position
import signal
import sys
import time
import math

v = 0
def signal_handler(signal, frame):
    global v
    print('Exiting controller')
    v.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
v = Position.ViconPosition("Flamewheel@192.168.20.10")
#v = FakePosition.FakePosition(1.2, 1.2, 1.2)
v.start()
rad2deg = 180 / math.pi
while (1):
    print 'Roll: %1.2f Pitch: %1.2f Yaw: %1.2f' % (v.angles[0]*rad2deg, v.angles[1]*rad2deg, v.angles[2]*rad2deg)
    time.sleep(0.1)




