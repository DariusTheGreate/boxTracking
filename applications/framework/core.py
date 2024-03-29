import os
import time
import cv2 as cv  # pip install opencv-python
import math
import time
import numpy as np
import statistics
import datetime


frameRate = 4
deltaBetweenControl = 2

showImageMode = False

class ControlState():
    def __init__(self):
        self.time_list = []
        self.last_milliseconds_event_time = 0
        #self.propriet_vector_list = []


class Box():
    def __init__(self, left, lower, right, up):
        self.left = left
        self.lower = lower
        self.right = right
        self.up = up

    def paintBox(self, frame):
        cv.rectangle(frame, (self.left, self.lower), (self.right, self.up), (255,0,255), 3)

    def getCroppedFrame(self, frame):
        return frame[self.up:self.lower, self.left:self.right]

class Frame():
    def __init__(self, img):
        self.img = img
        self.orb = cv.ORB_create() 

    def getFeaturesPoints(self): 
        if self.img is not None:
            self.img = cv.medianBlur(self.img, 3)
            #self.showFrame()
            feats = cv.goodFeaturesToTrack(np.mean(self.img, axis=2).astype(np.uint8), 3000, qualityLevel=0.01, minDistance=3) 
            kps = [cv.KeyPoint(x=f[0][0], y=f[0][1], size=20) for f in feats] 
            kps, des = self.orb.compute(self.img, kps)

            return (kps, des)

    def showFrame(self):
        if showImageMode:
            return
        cv.imshow("frame", self.img)
        #cv.waitKey(0)


class FeatureBasedMotionDetector():
    def __init__(self, frame_img1, frame_img2):
        self.frame1 = Frame(frame_img1)
        self.frame2 = Frame(frame_img2)

    def getMotionDeltaVector(self):
        pm, kp1, kp2 = self.compareTwoFrame()
        curr_dx, curr_dy = self.checkMotionInFeturesBox(pm, kp1, kp2)
        return curr_dx, curr_dy

    def getMotionStatus(self):
        pm, kp1, kp2 = self.compareTwoFrame()
        curr_dx, curr_dy = self.checkMotionInFeturesBox(pm, kp1, kp2)
        
        status = False

        if (abs(curr_dx) > 0):
            status = True

        if (abs(curr_dy) > 0):
            status = True

        return status

    def compareTwoFrame(self):
        bf = cv.BFMatcher(cv.NORM_HAMMING) 
        
        perfectMatches = []
        kp1 = []
        kp2 = []
        #frame1.showFrame()
        #frame2.showFrame()
        
        features1 = self.frame1.getFeaturesPoints() 
        features2 = self.frame2.getFeaturesPoints() 
        
        if features1[1] is None or features2[1] is None:
            return perfectMatches, kp1, kp2

        matches = bf.knnMatch(features1[1], features2[1], k=2)
        if len(matches[0]) == 1:
            return perfectMatches, kp1, kp2

        retdescs = [] 
        for m, n in matches:
            if m.distance < 0.75 * n.distance:
                retdescs.append(m)

        kp1 = features1[0]        
        kp2 = features2[0]
            
        for mat in retdescs:
            img1_idx = mat.queryIdx
            img2_idx = mat.trainIdx
            (x1, y1) = kp1[img1_idx].pt
            (x2, y2) = kp2[img2_idx].pt
            if abs(x2 - x1) < 0.1 and abs(y2 - y1) < 0.1:
                continue
            perfectMatches.append(mat)
        #print(len(perfectMatches), len(kp1), len(kp2))
        return perfectMatches, kp1, kp2

    def checkMotionInFeturesBox(self, perfectMatches, kp1, kp2):
        motionFeaturesDx = []
        motionFeaturesDy = []
        
        for mat in perfectMatches:
            img1_idx = mat.queryIdx
            img2_idx = mat.trainIdx

            if img1_idx < len(kp1):
                (x1, y1) = kp1[img1_idx].pt
            if img2_idx < len(kp2):
                (x2, y2) = kp2[img2_idx].pt
                
            #if x1 > 0 and x2 < 0:
            motionFeaturesDx.append(x2 - x1)
            motionFeaturesDy.append(y2 - y1)

        return np.median(motionFeaturesDx), np.median(motionFeaturesDy)


class ContourColorBasedMotionDetector():
    def __init__(self, lower_color, higher_color, contour_square_threshold, frame_img):
        self.lower_color = lower_color
        self.higher_color = higher_color
        self.contour_square_threshold = contour_square_threshold
        self.frame = Frame(frame_img)

    def getMotionStatus(self):
        hsv = cv.cvtColor(self.frame.img, cv.COLOR_BGR2HSV)
        color_range = cv.inRange(hsv, self.lower_color, self.higher_color)
        color_range_frame = Frame(color_range)
        #color_range_frame.showFrame()
        contours, hierarchy = cv.findContours(color_range, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
        if (len(contours) > 0):
            res_contour = cv.contourArea(max(contours, key = cv.contourArea))
            if res_contour > self.contour_square_threshold:
                return True
        return False


class AbsDiffBasedMotionDetector():
    def __init__(self, frame_img1, frame_img2):
        self.frame1 = Frame(frame_img1)
        self.frame2 = Frame(frame_img2)

    def getMotionStatus(self):
        diff = cv.absdiff(self.frame1.img, self.frame2.img)
        diff_frame = Frame(diff)
        diff_mean = np.mean(diff)
        #diff_frame.showFrame()
        if (diff_mean > 10):
            return True
        return False


class Controller():
    def __init__(self, control_state, id, time_dist = 6, event_counter = 5, time_delta_between_event = 3):
        self.control_state = control_state
        self.id = id #disable?
        self.time_dist = time_dist
        self.event_counter = event_counter
        self.time_delta_between_event = time_delta_between_event

    def controlInListMode(self, frame_img, millis):
        time_array = self.control_state.time_list
        out = str(int(millis/1000))
        time_array.append(out)        
        self.controlList(millis)    

    def controlInBaseMode(self, frame_img, millis):
        self.handleEvent(time, millis)

    def controlList(self, millis):

        log_array = self.control_state.time_list

        
        if len(log_array) == 0:
            return
        if millis/1000 - float(log_array[-1]) <= deltaBetweenControl:
            return

        start = 0
        prev = float(log_array[0])
        length = 0
        for i in log_array:
            if (float(i) - prev) > self.time_dist:
                #print("big dist ", (float(i) - prev), dist)
                length += 1
                break
            length += 1
            prev = float(i)
        if length > self.event_counter:
            interval = [float(v) for v in log_array[start:length]]
            time = datetime.timedelta(seconds=statistics.median(interval))
            self.handleEvent(time, millis)

        del log_array[start:length]

    def handleEvent(self, time, millis):
        print(self.id, str(time), "время логирования -> ", str(datetime.timedelta(milliseconds = int(millis)))) 
        print()


class MotionContourDetectorController(Controller):
    def __init__(self, control_state, id, controlBox, contour_square_threshold, lower_color = [0,0,180], higher_color = [255,255,255], handle_function =None, time_dist = 6, event_counter = 5, time_delta_between_event = 3000):
        self.control_state = control_state
        self.id = id #disable?
        self.controlBox = controlBox
        self.contour_square_threshold = contour_square_threshold
        self.lower_color = np.array(lower_color)
        self.higher_color = np.array(higher_color)
        self.time_dist = time_dist
        self.event_counter = event_counter
        self.handle_function = handle_function
        self.time_delta_between_event = time_delta_between_event

    def controlInListMode(self, frame_img, millis):
        contour_analyser = ContourColorBasedMotionDetector(self.lower_color, self.higher_color, self.contour_square_threshold, self.controlBox.getCroppedFrame(frame_img))
        contour_status = contour_analyser.getMotionStatus()
        if (contour_status):
            out = str(int(millis/1000))
            self.control_state.time_list.append(out)
            
        super().controlList(millis)   

    def controlInBaseMode(self, frame_img, millis):
        contour_analyser = ContourColorBasedMotionDetector(self.lower_color, self.higher_color, self.contour_square_threshold, self.controlBox.getCroppedFrame(frame_img))
        contour_status = contour_analyser.getMotionStatus()

        if (contour_status):
            
            out = str(datetime.timedelta(milliseconds = int(millis)))
            #print(int(millis), self.control_state.last_milliseconds_event_time, millis - self.control_state.last_milliseconds_event_time, millis - self.control_state.last_milliseconds_event_time > self.time_delta_between_event )
            if (millis - self.control_state.last_milliseconds_event_time > self.time_delta_between_event):
                self.handleEvent(out, millis)
            self.control_state.last_milliseconds_event_time = millis

    def handleEvent(self, time, millis):
        if (self.handle_function is not None):
            self.handle_function(time, millis) 
        else:
            super().handleEvent(time, millis)


class MotionFeatureDetectorController(Controller):
    def __init__(self, control_state, id, controlBox, dx_threshold, dy_threshold, handle_function = None, time_dist = 6, event_counter = 5, time_delta_between_event = 3000):
        self.control_state = control_state
        self.id = id #disable?
        self.controlBox = controlBox
        self.dx = dx_threshold
        self.dy = dy_threshold
        self.time_dist = time_dist
        self.event_counter = event_counter
        self.handle_function = handle_function
        self.time_delta_between_event = time_delta_between_event

    def controlInListMode(self, frame_img1, frame_img2, millis):

        feture_detection_analyser = FeatureBasedMotionDetector(self.controlBox.getCroppedFrame(frame_img1), self.controlBox.getCroppedFrame(frame_img2))
        curr_dx, curr_dy = feture_detection_analyser.getMotionDeltaVector()

        contour_analyser = ContourColorBasedMotionDetector(np.array([0,0,180]), np.array([255,255,255]), 6000, self.controlBox.getCroppedFrame(frame_img2))
        contour_status = contour_analyser.getMotionStatus()

        if (not np.isnan(curr_dx) and abs(curr_dx) > self.dx) and (not np.isnan(curr_dy) and abs(curr_dy) > self.dy):
            out = str(int(millis/1000))
            self.control_state.time_list.append(out)
            
        super().controlList(millis)    

    def controlInBaseMode(self, frame_img1, frame_img2, millis):

        diff_analyser = AbsDiffBasedMotionDetector(self.controlBox.getCroppedFrame(frame_img1), self.controlBox.getCroppedFrame(frame_img2))
        diff_status = diff_analyser.getMotionStatus()

        if not diff_status:
            return

        feture_detection_analyser = FeatureBasedMotionDetector(self.controlBox.getCroppedFrame(frame_img1), self.controlBox.getCroppedFrame(frame_img2))
        curr_dx, curr_dy = feture_detection_analyser.getMotionDeltaVector()

        contour_analyser = ContourColorBasedMotionDetector(np.array([0,0,180]), np.array([255,255,255]), 6000, self.controlBox.getCroppedFrame(frame_img2))
        contour_status = contour_analyser.getMotionStatus()
        
        if (not np.isnan(curr_dx) and abs(curr_dx) > self.dx) and (not np.isnan(curr_dy) and abs(curr_dy) > self.dy) and (contour_status) and (diff_status):
            out = str(datetime.timedelta(milliseconds = int(millis)))

            if (millis - self.control_state.last_milliseconds_event_time > self.time_delta_between_event):
                self.handleEvent(out, millis)
            self.control_state.last_milliseconds_event_time = millis

    def handleEvent(self, time, millis):
        if (self.handle_function is not None):
            self.handle_function(time, millis)
