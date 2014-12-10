import pid
import time
import numpy as np
import numpy.matlib as matlib 
import matplotlib.pyplot as plt

refCmd = 1300
initCond = 10

mypid = pid.PidController(5.0, 2, 0.0, 0, 1600)
mypid.set(refCmd)


process_val = initCond
timeLen = 100
x = np.zeros(timeLen)
output = np.zeros(timeLen)
for i in range(0,timeLen):

    mypid.step(process_val)
    process_val = process_val + mypid.get()*0.05
    print 'Reference: %d Plant: %d' % (refCmd, process_val)
    x[i] = i
    output[i] = process_val
    #time.sleep(0.5)

line, = plt.plot(x, output)
plt.show()

