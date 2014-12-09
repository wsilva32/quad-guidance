import cv2,cv
import numpy as np
import time
import threading

class BalloonFinder(threading.Thread):
    def __init__(self, m_width=0, m_height=0):
        super(BalloonFinder, self).__init__()
        self._stop = threading.Event()
        self.vidSize = (m_width, m_height)
        self.centerX = int(m_width/2)
        self.centerY = int(m_height/2)
        self.area = 0
        self.centroid = (0,0)
        self.frameRate = 0.0

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def findBalloon(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h,s,v = cv2.split(hsv)

        #openCV wraps hue values from 180 to 0: select the non-range, and then invert to get the 
        #range we want

        hsvLower = (8,149,67)
        hsvUpper = (175,255,255)


        inv_mask_h = cv2.inRange(h, np.array([hsvLower[0]], dtype=np.uint8), np.array([hsvUpper[0]], dtype=np.uint8))

        mask = 255 - inv_mask_h

        kernel_four = np.ones((4,3),np.uint8)
        kernel_five = np.ones((5,5),np.uint8)
        kernel_ten = np.ones((10,10),np.uint8)

        mask = cv2.erode(mask, kernel_four, iterations=2)
        mask = cv2.dilate(mask, kernel_five, iterations=1)
        
        # mask = cv2.dilate(mask, kernel_five)
        # mask = cv2.erode(mask, kernel_five)
        #res = cv2.bitwise_and(frame,frame, mask= mask)
       
        contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frame, contours, -1, (0,255,0), 3)
        #print contours
        try:
                if len(contours) == 0:
                        return None

                maxArea = 0;
                cnt = None

                #pick the biggest one to track
                for theCnt in contours:
                        #print 'Contour:', i
                        theArea = cv2.contourArea(theCnt)
                        if theArea > maxArea:
                                cnt = theCnt
                                maxArea = theArea
                                
                area = maxArea
                #print 'Max area: ', area
                #perimeter = cv2.arcLength(cnt, True)
                
                moment = cv2.moments(cnt)
                cx = int(moment['m10']/moment['m00'])
                cy = int(moment['m01']/moment['m00'])

                min_con_x = min(x[0][0] for x in cnt)
                min_con_y = min(y[0][1] for y in cnt)
                max_con_x = max(x[0][0] for x in cnt)
                max_con_y = max(y[0][1] for y in cnt)

                out_of_bounds = False
                if min_con_x == 1 or min_con_y == 1 or max_con_x == 638 or max_con_y == 478:
                        out_of_bounds = True

                num_objects = len(contours)
                #print 'Num objects:',num_objects
                image_data = [cx ,cy, area, min_con_x, max_con_x, min_con_y, max_con_y, out_of_bounds, num_objects]

        except:
                #print 'Got exception'
                area = None
                perimeter = None
                cx = None
                cy = None
                min_con_x = None
                min_con_y = None
                max_con_x = None
                max_con_y = None
                out_of_bounds = None
                image_data = None
        return image_data

    def run(self):
        cap = cv2.VideoCapture(0)
        cap.set(cv.CV_CAP_PROP_FRAME_WIDTH, self.vidSize[0])
        cap.set(cv.CV_CAP_PROP_FRAME_HEIGHT, self.vidSize[1])
        time.sleep(1)
        getMilliTime = lambda: int(round(time.time() * 1000))
        lastTime = getMilliTime()

        while not self.stopped():
            _,frame = cap.read()
            
            imgData = self.findBalloon(frame)
            if imgData:
                self.area = imgData[2]
                self.centroid = (imgData[0]-self.centerX, imgData[1]-self.centerY)
                #print 'Area of max: %d Centroid (%d,%d)' % (imgData[2], imgData[0], imgData[1])
                
            loopTime = getMilliTime()
            #print 'Elapsed ms/frame: %s' % (loopTime - lastTime)
            self.frameRate = 1 / float(loopTime - lastTime) * 1000

            lastTime = loopTime
