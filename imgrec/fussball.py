import cv
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
			self.cap = cv.CreateCameraCapture(0)
		else:
			filename = 'out.avi'
			self.cap = cv.CreateFileCapture(filename)
			if not self.cap  :
				raise "file load fail!"
			else:
				print "Frame: " + str(cv.GetCaptureProperty(self.cap, cv.CV_CAP_PROP_POS_FRAMES ))
				print "Frame Height: " + str(cv.GetCaptureProperty(self.cap, cv.CV_CAP_PROP_FRAME_HEIGHT  ))
				print "FPS: " + str(cv.GetCaptureProperty(self.cap, cv.CV_CAP_PROP_FPS  ))
				print "FourCC: " + str(cv.GetCaptureProperty(self.cap, cv.CV_CAP_PROP_FOURCC ))
			frame = cv.QueryFrame(self.cap)

		self.playmask = cv.LoadImage("Table.png")
		self.polemask = cv.LoadImage("Poles.png")
		self.linemask = cv.LoadImage("Lines.png")

		# create some buffers
		self.yuvframe = cv.CreateMat(frame.height, frame.width, cv.CV_8UC3)
		self.imgThreshed = cv.CreateMat(frame.height, frame.width, cv.CV_8UC1)
		self.imgDilated =  cv.CreateMat(frame.height, frame.width, cv.CV_8UC1)
		self.maskedFrame = cv.CreateMat(frame.height, frame.width, cv.CV_8UC3)
		self.storage = cv.CreateMemStorage(0)


	def init_interactive(self):
		cv.NamedWindow('Table', cv.CV_WINDOW_AUTOSIZE)
		#cv.NamedWindow('h', cv.CV_WINDOW_AUTOSIZE)
		cv.NamedWindow('s', cv.CV_WINDOW_AUTOSIZE)
		#cv.NamedWindow('l', cv.CV_WINDOW_AUTOSIZE)

	def __del__(self):
		cv.DestroyAllWindows()

	def update(self, ball ):
		key = -1
		key = cv.WaitKey(5) # get user input


		frame = cv.QueryFrame(self.cap)
		if frame != None:
			bigblob = self.locateBlobs(frame)

				#minBallDiameter = 5
			CV_RED = cv.Scalar(0,0,255)
			if bigblob != None:
				cv.Circle(frame, (int(bigblob[1][0]), int(bigblob[1][1])), int(bigblob[2]), CV_RED)

			#if key == ord('s'):
			#	saveFrame(frame)

		if key == ord('q') and self.interactive : # 'q'
			return -1
		if key == ord('j') and not self.live and self.interactive :
			cv.SetCaptureProperty(self.cap, cv.CV_CAP_PROP_POS_FRAMES, cv.GetCaptureProperty(self.cap, cv.CV_CAP_PROP_POS_FRAMES) + 400)
		if key == ord('k') and not self.live and self.interactive :
			cv.SetCaptureProperty(self.cap, cv.CV_CAP_PROP_POS_FRAMES, cv.GetCaptureProperty(self.cap, cv.CV_CAP_PROP_POS_FRAMES) - 400)
		if self.interactive:
			cv.ShowImage('Table', frame) # show the image
		return 0


	def locateBlobs(self, frame):

		#load mask
		cv.And(frame, self.playmask, self.maskedFrame ) # mask off play area
		cv.And(self.maskedFrame, self.polemask, self.maskedFrame ) # mask off play area
		cv.And(self.maskedFrame, self.linemask, self.maskedFrame ) # mask off play area

		#gaussian blur
		cv.Smooth(self.maskedFrame,self.maskedFrame, cv.CV_GAUSSIAN,3,3)



		#imgContour =  cv.CreateMat(frame.height, frame.width, cv.CV_8UC1) # scratch space for findcontours


		#convert to HLS colour space
		cv.CvtColor(self.maskedFrame, self.yuvframe, cv.CV_RGB2HLS)
		# threshold on saturation being high
		cv.InRangeS(self.maskedFrame, cv.Scalar(0, 0, 200), cv.Scalar(255, 255, 255), self.imgThreshed)


		#dilate the Thresholded image
		cv.Dilate(self.imgThreshed, self.imgDilated)

		# copy dialated int
		#cv.Copy(imgDilated, imgContour)
		contours = cv.FindContours(self.imgDilated,  self.storage)



		maxContour = None
		maxContourArea = 0

		#contours.h_next()
		try:
			while contours != None:
				# get points
				circle = cv.MinEnclosingCircle(contours)
				if maxContourArea < cv.ContourArea(contours):
					maxContour = contours
					maxContourArea =  cv.ContourArea(contours)
				contours = contours.h_next()
			circle = cv.MinEnclosingCircle(maxContour)
			return circle
		except:
			print 'meh'
			return None


		# just return the biggest circle

	def saveFrame(self, frame):
		cv.SaveImage("template.png", frame)


if __name__ == "__main__":
	table = fussball()
	table.play()
