
import threading
import math
import time

class FakePosition(threading.Thread):
    def __init__(self, m_x, m_y, m_z):
        super(FakePosition, self).__init__()
        self._stop = threading.Event()
        self.position = (m_x,m_y,m_z)
        self.angles = (0,0,0)

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        while not self.stopped():
            time.sleep(0.01)

