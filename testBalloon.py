import camera.BalloonFinder
import signal
import sys
import time

cam = 0
def signal_handler(signal, frame):
    global cam
    print('Exiting controller')
    cam.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
cam = camera.BalloonFinder.BalloonFinder(320,240)
cam.start()
print 'Current video size: %dx%d' % (cam.vidSize[0], cam.vidSize[1])
while (1):
    print 'Area: %d Centroid: (%d,%d) FPS: %1.2f' % (cam.area, cam.centroid[0], cam.centroid[1], cam.frameRate)
    time.sleep(0.1)
