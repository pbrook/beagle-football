#! /usr/bin/python -tt

import numpy as np
import cv2
import time
import math
#import sys

class ball():
    def __init__(self, size):
        self.x = 0
        self.y = 0
        self.dx = 0
        self.dy = 0
        self.size = size
        self.last_err = 0
        self.last_update = None

    def get_pos(self, now):
        if self.last_update is None:
            return (self.x, self.y)
        dt = now - self.last_update
        return (self.x + dt * self.dx, self.y + dt * self.dy)

    def update(self, now, x, y, r):
        if self.last_update is None:
            self.x = x
            self.y = y
            self.last_update = now
            self.last_err = abs(self.size - r)
            return

        # Calculate the predicted location
        dt = now - self.last_update
        pred_x = self.x + dt * self.dx
        pred_y = self.y + dt * self.dy

        # See how far that is from the actual location
        err_x = pred_x - x
        err_y = pred_y - y
        # TODO: Maybe factor in dt? Maybe not.
        pred_err_dist = math.sqrt(err_x * err_x + err_y * err_y)
        # Use the observed ball size as a proxy for measurement accuracy
        # If we only see a small [section of] ball then it's partially
        # obscured, and probably not a very good guess.  All we really know
        # is that the ball covers that area
        new_err = abs(self.size - r)
        if pred_err_dist < (self.last_err + new_err) * 1.5:
            # We are reasonably close to the predicted location
            # Make incremental adjustment to estimated velocity
            # Todo factor in relative measurement errors
            self.dx += err_x * 0.5 / dt
            self.dy += err_y * 0.5 / dt
            self.x += dt * self.dx
            self.y += dt * self.dy
        else:
            # Not close to where we expected to be.  Start from scratch.
            self.dx = (x - self.x) / dt
            self.dy = (y - self.y) / dt
            self.x = x
            self.y = y
        self.last_err = new_err

class fussball():
    def __init__(self,  live = False, interactive = True):
        self.ball = ball(20)
        self.game = None
        self.cv = fussballcv( live, interactive)

    def play(self):
        status = 0
        while status >= 0:
            status = self.cv.update(self.ball)

class fussballcv():

    def __init__(self, live = False, interactive = True):
        self.live = live
        self.interactive = interactive

        if live:
            self.cap = cv2.VideoCapture(2)
        else:
            filename = 'out2.avi'
            self.cap = cv2.VideoCapture(filename)
            if not self.cap  :
                raise "file load fail!"
            else:
                print "Frame: " + str(self.cap.get(cv2.cv.CV_CAP_PROP_POS_FRAMES ))
                print "Frame Height: " + str(self.cap.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT  ))
                print "FPS: " + str(self.cap.get(cv2.cv.CV_CAP_PROP_FPS  ))
                print "FourCC: " + str(self.cap.get(cv2.cv.CV_CAP_PROP_FOURCC ))
            (retval, frame) = self.cap.read()

        self.playmask = cv2.imread("Table.png")
        self.polemask = cv2.imread("Poles.png")
        self.linemask = cv2.imread("Lines.png")
        self.all_mask = self.playmask & self.polemask #& self.linemask
        self.frame_time = 0.0

    def find_mask_position(self, frame):
        cv2.matchTemplate(frame, self.all_mask, cv2.TM_SQDIFF_NORMED)
        pass


    def init_interactive(self):
        cv2.NamedWindow('Table', cv.CV_WINDOW_AUTOSIZE)
        #cv.NamedWindow('h', cv.CV_WINDOW_AUTOSIZE)
        cv2.NamedWindow('thresh', cv.CV_WINDOW_AUTOSIZE)
        #cv.NamedWindow('l', cv.CV_WINDOW_AUTOSIZE)

    def update(self, ball):
        key = -1
        key = cv2.waitKey(20) # get user input


        t1 = time.time()
        if self.live:
            now = time.time()
        else:
            now = self.frame_time
            self.frame_time += 1.0/60
        (retval, frame) = self.cap.read()
        if frame != None:
            blob = self.locateBlobs(frame)

            #minBallDiameter = 5
            CV_RED = (0,0,255)
            CV_GREEN = (0,255,0)
            if blob != None:
                pos = blob[0]
                r = blob[1]
                ball.update(now, pos[0], pos[1], r)
                cv2.circle(self.foo, (int(pos[0]), int(pos[1])), int(r), CV_RED)
                cv2.circle(frame, (int(pos[0]), int(pos[1])), int(r), CV_RED)
            pos = ball.get_pos(now)
            cv2.circle(frame, (int(pos[0]), int(pos[1])), int(ball.size), CV_GREEN);
        t2 = time.time()

        #if key == ord('s'):
        #        saveFrame(frame)
        if key == ord('q') and self.interactive : # 'q'
            return -1
        if key == ord('j') and not self.live and self.interactive :
            cv2.SetCaptureProperty(self.cap, cv.CV_CAP_PROP_POS_FRAMES, cv.GetCaptureProperty(self.cap, cv.CV_CAP_PROP_POS_FRAMES) + 400)
        if key == ord('k') and not self.live and self.interactive :
            cv2.SetCaptureProperty(self.cap, cv.CV_CAP_PROP_POS_FRAMES, cv.GetCaptureProperty(self.cap, cv.CV_CAP_PROP_POS_FRAMES) - 400)
        if self.interactive:
            cv2.imshow('Table', frame) # show the image
            cv2.imshow('thresh', self.foo)
        print 1/(t2 - t1)
        return 0


    def locateBlobs(self, frame):

        #load mask
        maskedFrame = frame & self.all_mask;

        #gaussian blur
        cv2.GaussianBlur(maskedFrame, (3, 3), 0, maskedFrame)


        #convert to HLS colour space (slow, not currently used)
        #yuvframe = cv2.cvtColor(maskedFrame, cv2.COLOR_RGB2HLS)
        # threshold on saturation being high

        low = np.array([0x80, 0x80, 0xb0], np.uint8)
        high = np.array([0xcc, 0xdf, 0xff], np.uint8)
        #low = np.array([200, 200, 200], np.uint8)
        #high = np.array([255, 255, 255], np.uint8)
        threshframe = cv2.inRange(maskedFrame, low, high)
        self.foo = threshframe.copy()

        #dilate the Thresholded image
                        #cv2.dilate(threshframe, None, dst=threshframe)
        cv2.erode(threshframe, None, dst=threshframe)
        #self.t = threshframe.copy()

        contours, hierarchy = cv2.findContours(threshframe, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)



        maxContour = None
        maxContourArea = 0

        for c in contours:
            # get points
            circle = cv2.minEnclosingCircle(c)
            area = cv2.contourArea(c)
            if maxContourArea < area:
                maxContour = c
                maxContourArea = area
        if maxContour is None:
            return None
        circle = cv2.minEnclosingCircle(maxContour)
        # just return the biggest circle
        return circle

    def saveFrame(self, frame):
        cv.imsave("template.png", frame)



if __name__ == "__main__":
    table = fussball( live = False, interactive = True)
    table.play()
