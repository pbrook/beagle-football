#! /usr/bin/python -tt

import numpy as np
import cv2
import time
import math
#import sys

import pru_stick

record = False
display_mask = False

# How long it takes to raise the player for a kick
KICK_MIN_TIME = 0.5

class Ball(object):
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
            self.last_update = now
            self.x = x
            self.y = y
            self.last_err = abs(self.size - r)
            return

        # Calculate the predicted location
        dt = now - self.last_update
        self.last_update = now
        pred_x = self.x + dt * self.dx
        pred_y = self.y + dt * self.dy

        # See how far that is from the actual location
        err_x = x - pred_x
        err_y = y - pred_y
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
            # TODO: factor in relative measurement errors
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

SMALL_SPEED = 1
def do_intercept(pos, vel, x):
    if vel[0] < SMALL_SPEED:
        return (None, None)
    m = vel[1] / vel[0]
    dx = x - pos[0]
    dy = dx * m
    y = pos[1] + dy
    t = dx / vel[0]
    return (y, t)

# 26 - (0)red goal, (1)red defence, (2)blue attack, (3)red mid, (4)blue mid
# (5)red attack, (6)blue defence, (7)blue goal - 294
# We are blue
red_goal_pos = 26
pole_pos = [40, 74, 108, 143, 177, 212, 246, 280]
# Goal, Midfield, Attack
# TODO: Midfeld values are wrong
# Maximum (absolute) stick offset
stick_limit = [24, 24, 24]
# Offset at which player regions overlap
stick_player_spacing = [22, 22, 22]
blue_goal_pos = 296

class PRUDriver(object):
    def __init__(self, mon):
        self.mon = mon
        self.cur_set_pos = [0.0]*3
        self.cur_rot = [True]*3
        self.pos_range = [0]*3
        self.mode = 0

    def set_rot(self, n, down):
        if (n < 0) or (n > 2):
            return
        self.cur_rot[n] = down

    def set_pos(self, n, pos):
        if (n < 0) or (n > 2):
            return
        if pos > 1.0:
            pos = 1.0
        elif pos < 0.0:
            pos = 0.0
        self.cur_set_pos[n] = pos;

    def check_init(self):
        val = self.mon.poll(pru_stick.COM_RANGE)
        self.mode = val >> 24
        if self.mode == 2:
            self.pos_range[0] = val & 0xff
            self.pos_range[1] = (val >> 8) & 0xff
            self.pos_range[2] = (val >> 12) & 0xff

    def sync(self):
        if self.mode != 2:
            self.check_init()
            return
        newpos = 0
        for n in xrange(0, 3):
            b = int(self.cur_set_pos[n] * self.pos_range[n]) + 0x80
            newpos |= b << (n * 8)
            if self.cur_rot[n]:
                newpos |= 1 << (24 + n)
        self.mon.write32(pru_stick.COM_SET_POS, newpos)

class StickController(object):
    def __init__(self, pd, stick_num, ball, x, next_x):
        self.ball = ball
        self.x = x
        self.center = 130
        self.offset = 0
        self.limit = stick_limit[stick_num]
        self.spacing = stick_player_spacing[stick_num]
        self.kick_zone = next_x + ball.size
        self.down = True
        self.pd = pd
        self.stick_num = stick_num

    def intercept(self, pos):
        vel = (self.ball.dx, self.ball.dy)
        (y, t) = do_intercept(pos, vel, self.x)
        if y is None:
            return y
        return self.find_dy(y)

    # Find the vertical delta between the requested position
    # and our current position
    def find_dy(self, y):
        dy = y - (self.center + self.offset)
        # Select the nearest player
        while dy > self.spacing:
            dy -= self.spacing * 2
        while dy < self.spacing:
            dy += self.spacing * 2
        # Check we have enough range to get there
        if self.offset + dy > self.limit:
            dy -= self.spacing * 2
        elif self.offset + dy < -self.limit:
            dy += self.spacing * 2
        return dy

    def rotate(self, down):
        self.down = down
        self.pd.set_rot(self.stick_num, self.down)

    def lower(self):
        self.rotate(True)

    def kick(self):
        self.rotate(True)

    def lift(self):
        self.rotate(False)

    def move(self, offset):
        if offset > self.limit:
            offset = self.limit
        elif offset < -self.limit:
            offset = -self.limit
        self.offset = offset
        self.pd.set_pos(self.stick_num, offset / float(self.limit))

    def move_block(self, x, y):
        # Position ourselves between the ball and the middle of the goal
        # TODO: Sometimes better to be in the ball's path
        self.lower()
        dx = x - blue_goal_pos
        dy = y - self.center
        my_dx = self.x - blue_goal_pos
        my_y = self.center + my_dx * dy / dx
        my_dy = self.find_dy(my_y)
        self.move(self.offset + my_dy)

    def update(self, now):
        # Assumes we are playing towards negative x
        dia = self.ball.size
        (x, y) = self.ball.get_pos(now)
        # delta_y lines us up with the current ball position
        delta_y = self.find_dy(self.ball.y)
        if x > self.x + dia:
            print 'Behind'
            # ball is behind us,  move out of the way
            self.lift()
            self.move(self.offset + delta_y)
        elif x > self.x - (dia * 0.8):
            # TODO: factor in the ball velocity
            if abs(delta_y) < dia:
                print 'Kick', self.x - x
                # Ball immediately underneath us
                self.kick()
            else:
                print 'Pass', int(delta_y), int(self.offset)
        elif (self.ball.dx > 0) and (x >= self.kick_zone):
            inter_dy = self.intercept((x, y))
            # line up with the ball trajectory
            if inter_dy is None:
                print 'Slow intercept'
                inter_dy = delta_y
            else:
                print 'Intercept'
            self.move(self.offset + inter_dy)
            if self.ball.dx * KICK_MIN_TIME > self.x - self.kick_zone:
                # The ball is travelling quicky
                # Try to block it
                self.lower()
            else:
                # Raise ready to kick
                self.lift()
        else:
            # Ball moving away from us or far away
            print 'Defend'
            # For simplicity, just line up with the ball
            #self.move_block(x, y)
            self.lower()
            self.move(self.offset + delta_y)

    def render(self, frame):
        if self.down:
            color = (0,0,255)
        else:
            color = (255,0,255)
        cv2.circle(frame, (self.x, int(self.center + self.offset)), 4, thickness = 4, color=color)

class fussball(object):
    def __init__(self, res, live, interactive):
        self.ball = Ball(10.0)
        self.game = None
        self.stick = []
        self.pru_mon = pru_stick.PRUMonitor("./pru_stick.bin")
        self.pd = PRUDriver(self.pru_mon)
        # Attack
        self.stick.append(StickController(self.pd, 2, self.ball, pole_pos[2], pole_pos[1]))
        # Midfield
        self.stick.append(StickController(self.pd, 1, self.ball, pole_pos[4], pole_pos[3]))
        # Goal
        self.stick.append(StickController(self.pd, 0, self.ball, pole_pos[7], pole_pos[5]))
        self.cv = ImgRec(res, live, interactive, self.render)

    def render(self, frame):
        for s in self.stick:
            s.render(frame)

    def play(self):
        status = 0
        while status >= 0:
            status = self.cv.update(self.ball)
            if self.cv.live:
                now = time.time()
            else:
                now = self.cv.frame_time
            for s in self.stick:
                s.update(now)

class ImgRec(object):

    def __init__(self, res, live, interactive, rendercb):
        self.live = live
        self.interactive = interactive
        self.res = res
        self.rendercb = rendercb

        self.need_resize = False
        if live:
            self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, res[0])
            self.cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, res[1])
            self.cam_fps = self.cap.get(cv2.cv.CV_CAP_PROP_FPS)
            if self.cam_fps <= 0:
                print "Calibrating camera..."
                # The capture driver can take a few frames to settle down
                for i in xrange(0, 5):
                    (retval, frame) = self.cap.read()
                # Figure out the frame rate
                start = time.time()
                frames = 0
                while time.time() < start + 1.0:
                    (retval, frame) = self.cap.read()
                    frames += 1
                # Round up
                self.cam_fps = frames + 1
            print "%d FPS" % self.cam_fps
            self.cap_time = time.time()
            self.buffer_frames = 0.0
        else:
            filename = 'in.avi'
            self.cap = cv2.VideoCapture(filename)
            if not self.cap  :
                raise "file load fail!"
            else:
                self.cam_fps = int(self.cap.get(cv2.cv.CV_CAP_PROP_FPS))
                print "Frame: " + str(self.cap.get(cv2.cv.CV_CAP_PROP_POS_FRAMES ))
                print "Frame Height: " + str(self.cap.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT  ))
                print "FPS: " + str(self.cap.get(cv2.cv.CV_CAP_PROP_FPS  ))
                print "FourCC: " + str(self.cap.get(cv2.cv.CV_CAP_PROP_FOURCC ))
                if res[0] != int(self.cap.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH)):
                    self.need_resize = True
                if res[1] != int(self.cap.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT)):
                    self.need_resize = True

        if record:
            self.writer = cv2.VideoWriter("out.avi", cv2.cv.CV_FOURCC('M', 'J', 'P', 'G'), 30, res, True)

        self.playmask = cv2.imread("Table.png")
        # no need to mask lines or poles when we can do color matching on the ball
        #self.polemask = cv2.imread("Poles.png")
        #self.linemask = cv2.imread("Lines.png")
        self.all_mask = self.playmask
        self.all_mask = cv2.resize(self.all_mask, res)
        self.frame_time = 0.0
        self.mark_time = 0.0

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
        key = cv2.waitKey(5) # get user input

        t1 = time.time()
        if self.live:
            now = time.time()
            # The capture driver seems to buffer several frames.
            # We can't process them at full rate, causing frames to back up,
            # and very high latency
            # Hack round this by keeping track of how many frames we think the
            # camera should have generated, and pull out any extras
            self.buffer_frames += (now - self.cap_time) * self.cam_fps
            slurp_frames = max(int(self.buffer_frames), 0)
            self.buffer_frames -= slurp_frames + 1
            if self.buffer_frames < -1.0:
                self.buffer_frames = 0.0
            for i in xrange(0, slurp_frames):
                (retval, frame) = self.cap.read()
            now = time.time()
            self.cap_time = now;
            #print "Real: %2d %.2f" % (int(1.0/(now - self.frame_time)), self.mark_time)
        else: # pre-recorded
            now = self.frame_time + 1.0/self.cam_fps
        (retval, frame) = self.cap.read()
        self.frame_time = now;

        if frame is None:
            return -1

            self.cap_time = now;
        if self.need_resize:
            frame = cv2.resize(frame, self.res)

        blob = self.locateBlobs(frame)

        #minBallDiameter = 5
        CV_RED = (0,0,255)
        CV_GREEN = (0,255,0)
        if blob != None:
            pos = blob[0]
            r = blob[1]
            ball.update(now, pos[0], pos[1], r)
            cv2.circle(frame, (int(pos[0]), int(pos[1])), int(r), CV_RED)
        (x, y) = ball.get_pos(now)
        x = int(x)
        y = int(y)
        #cv2.circle(frame, (x, y), int(ball.size), CV_GREEN, thickness=2);
        x2 = x + int(ball.dx/5)
        y2 = y + int(ball.dy/5)
        cv2.line(frame, (x, y), (x2, y2), CV_GREEN, thickness=2);
        self.rendercb(frame)
        t2 = time.time()

        #if key == ord('s'):
        #        saveFrame(frame)
        if key == ord(' '):
            if self.mark_time < 100.0:
                self.mark_time = self.frame_time
            else:
                self.mark_time = self.frame_time - self.mark_time
        if key == ord('q') and self.interactive : # 'q'
            return -1
        if key == ord('j') and not self.live and self.interactive :
            cv2.SetCaptureProperty(self.cap, cv.CV_CAP_PROP_POS_FRAMES, cv.GetCaptureProperty(self.cap, cv.CV_CAP_PROP_POS_FRAMES) + 400)
        if key == ord('k') and not self.live and self.interactive :
            cv2.SetCaptureProperty(self.cap, cv.CV_CAP_PROP_POS_FRAMES, cv.GetCaptureProperty(self.cap, cv.CV_CAP_PROP_POS_FRAMES) - 400)
        if self.interactive:
            cv2.imshow('Table', frame) # show the image
            if display_mask:
                cv2.imshow('thresh', self.foo)
        if record:
            self.writer.write(frame);
        #print "Virt: %d" % int(1.0/(t2 - t1))
        return 0


    def locateBlobs(self, frame):

        #load mask
        maskedFrame = frame & self.all_mask;

        #gaussian blur
        #cv2.GaussianBlur(maskedFrame, (3, 3), 0, maskedFrame)


        #convert to HLS colour space (slow, not currently used)
        #yuvframe = cv2.cvtColor(maskedFrame, cv2.COLOR_RGB2HLS)
        # threshold on saturation being high

        # Would probably be more effective to transform into
        # HSV color-space, but two RGB windows seems pretty
        # effective, and probably about the same cost.
        # A single RGB window covering both bright and dark orange
        # ends up including too much other stuff.

        # These values are calibrated for daylight (with floodlight)
        # If adjusting for artificial light please preserve these valus
        low = np.array([0x28, 0x50, 0xa0], np.uint8)
        mid1 = np.array([0x50, 0x90, 0xff], np.uint8)
        mid2 = np.array([0x40, 0x60, 0xe0], np.uint8)
        high = np.array([0xa0, 0xc0, 0xff], np.uint8)
        mask1 = cv2.inRange(maskedFrame, low, mid1)
        mask2 = cv2.inRange(maskedFrame, mid2, high)
        threshframe = mask1 | mask2
        if display_mask:
            self.foo = threshframe.copy()

        #dilate the Thresholded image
        #cv2.dilate(threshframe, None, dst=threshframe)
        #cv2.erode(threshframe, None, dst=threshframe)
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
    table = fussball(res=(320, 240), live = False, interactive = True)
    table.play()
