#! /usr/bin/env python

import numpy as np
import cv2
import time
#import sys


class ball():
	def __init__(self):
		self.x = 0
		self.y = 0
		self.momentx = 0
		self.momenty = 0

	def update(self, x,y):
		self.x = x
		self.y = y

class fussball():
	def __init__(self):
		self.ball = ball()
		self.game = None
		self.cv = fussballcv()

	def play(self):
		status = 0
		while status >= 0:
			status = self.cv.update(self.ball)
		#self.ball.update(self.cv.getballpos())

class fussballcv():

	def __init__(self, live = False, interactive = True):
		self.live = live
		self.interactive = interactive

		if live:
			self.cap = cv2.VideoCapture(0)
		else:
			filename = 'out.avi'
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
                self.all_mask = self.playmask & self.polemask & self.linemask

	def init_interactive(self):
		cv2.NamedWindow('Table', cv.CV_WINDOW_AUTOSIZE)
		#cv.NamedWindow('h', cv.CV_WINDOW_AUTOSIZE)
		cv2.NamedWindow('s', cv.CV_WINDOW_AUTOSIZE)
		#cv.NamedWindow('l', cv.CV_WINDOW_AUTOSIZE)

	def update(self, ball ):
		key = -1
		key = cv2.waitKey(1) # get user input


                t1 = time.time()
                (retval, frame) = self.cap.read()
		if frame != None:
			bigblob = self.locateBlobs(frame)

				#minBallDiameter = 5
			CV_RED = (0,0,255)
			if bigblob != None:
				cv2.circle(frame, (int(bigblob[0][0]), int(bigblob[0][1])), int(bigblob[1]), CV_RED)

			#if key == ord('s'):
			#	saveFrame(frame)
                t2 = time.time()

		if key == ord('q') and self.interactive : # 'q'
			return -1
		if key == ord('j') and not self.live and self.interactive :
			cv.SetCaptureProperty(self.cap, cv.CV_CAP_PROP_POS_FRAMES, cv.GetCaptureProperty(self.cap, cv.CV_CAP_PROP_POS_FRAMES) + 400)
		if key == ord('k') and not self.live and self.interactive :
			cv.SetCaptureProperty(self.cap, cv.CV_CAP_PROP_POS_FRAMES, cv.GetCaptureProperty(self.cap, cv.CV_CAP_PROP_POS_FRAMES) - 400)
                if self.interactive:
			cv2.imshow('Table', frame) # show the image
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
                low = np.array([200, 200, 200], np.uint8)
                high = np.array([255, 255, 255], np.uint8)
		threshframe = cv2.inRange(maskedFrame, low, high)


		#dilate the Thresholded image
                cv2.dilate(threshframe, None, dst=threshframe)
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
	table = fussball()
	table.play()
