import Position
import signal
import sys

v = 0
def signal_handler(signal, frame):
    global v
    print('Exiting controller')
    v.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
v = Position.ViconPosition("Wand2@192.168.20.10")
v.start()
while (1):
    print 'Z: %1.4f' % v.position[2]





