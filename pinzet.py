import os
import time
import cv2 as cv  # pip install opencv-python
import math
import time
import numpy as np
from enum import Enum
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from PyQt5 import QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import pandas as pd
import threading
#from threading import Thread
import _thread as thread

frameRate = 4

leftFeaturesBoarder = 1000
rightFeaturesBoarder = 1470
upperFeaturesBoarder = 550
lowerFeaturesBoarder = 700

time_delta_for_single_box = 12


def log_contains_simmilar(log, val):
    for i in log:
        if abs(float(i) - float(val)) < time_delta_for_single_box:
            return True 
    return False


def log_into_file(val, filename):
    with open(filename, 'a') as the_file:
        the_file.write(val)


class Frame():
    def __init__(self, img):
        self.img = img
        self.orb = cv.ORB_create() 

    def getFeaturesPoints(self): 
        if self.img is not None:
            gray = cv.cvtColor(self.img, cv.COLOR_BGR2GRAY)
            dst = cv.cornerHarris(gray, 2, 3, 0.099)
            feats = cv.goodFeaturesToTrack(np.mean(self.img, axis=2).astype(np.uint8), 3000, qualityLevel=0.01, minDistance=3) 
            kps = [cv.KeyPoint(x=f[0][0], y=f[0][1], _size=20) for f in feats] 
            kps, des = self.orb.compute(self.img, kps)

            return (kps, des)

    def showFrame(self):
        cv.imshow("frame", self.img)
        #cv.waitKey(0)


def compareTwoFrame(frame1, frame2):
    bf = cv.BFMatcher(cv.NORM_HAMMING) 
    
    perfectMatches = []
    kp1 = []
    kp2 = []
    #frame1.showFrame()
    #frame2.showFrame()
    
    features1 = frame1.getFeaturesPoints() 
    features2 = frame2.getFeaturesPoints() 
    
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


def checkMotionInFeturesBox(perfectMatches, kp1, kp2):
    #if motionIsStarted == True:
    #    return
    motionFeaturesDx = []
    motionFeaturesDy = []
    #print("-----------------")
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


def control_box(left, lower, right, up, dx, dy, frame1, frame2, box_id, millis, log_array):
    f1 = Frame(frame1[up:lower, left:right])
    f2 = Frame(frame2[up:lower, left:right])
    pm, kp1, kp2 = compareTwoFrame(f1, f2)
    curr_dx, curr_dy = checkMotionInFeturesBox(pm, kp1, kp2)

    if (not np.isnan(curr_dx) and curr_dx > 4) or (not np.isnan(curr_dy) and curr_dy > 4):
        out = str(int(millis/1000))
        if not log_contains_simmilar(log_array, out):
            log_array.append(out)
            log_into_file(str(box_id) + " " + str(out) + " seconds" + "\n", "log.txt")
        print(box_id, log_array)

class Application():
    def __init__(self, videoPath):
        self.videoPath = videoPath
        self.framesList = []
        self.packets = []
        self.currentFrame = None
        self.packetThreads = []

    def run(self):
        left1 = 1060
        right1 = 1430
        up1 = 190 
        lower1 = 330
        cap = cv.VideoCapture(self.videoPath)
        

        #чуть подвинуть налево?
        left2 = left1 - 800
        right2 = right1 - 800
        up2 = up1
        lower2 = lower1

        left3 = left1 + 500
        right3 = right1 + 500
        up3 = up1 + 160
        lower3 = lower1 + 160
        
        left4 = left3
        right4 = right3
        up4 = up3 + 180
        lower4 = lower3 + 180
        

        first_box_log_array = []
        second_box_log_array = []
        third_box_log_array = []
        four_box_log_array = []
        
        
        if cap.isOpened():
            global prevFrame
            prevFrameRes, prevFrame = cap.read()
        self.currentFrame = prevFrame

        frameCounter = 1
        init = True
        sumLen = 0
        sumLen2 = sumLen + 410
        startLine = True
        linesFront = []
        initPackets = True
        while(cap.isOpened()):
            frameCounter += 1
        
            ret, frame = cap.read()
            frameInit = frame.copy()
            self.currentFrame = frameInit.copy()
            if ret == True and frameCounter % frameRate == 0:
                
                if init != True:
                    millis = cap.get(0)
                    '''
                    #control_box(left, lower, right, up, 10, 10, prevFrame.copy(), frame.copy(), "first line", millis, first_box_log_array)
                    t1 = threading.Thread(name="box1_thread", target=control_box(left1, lower1, right1, up1, 10, 10, prevFrame.copy(), frame.copy(), "first line", millis, first_box_log_array)) # <<-- here
                    t2 = threading.Thread(name="box2_thread", target=control_box(left2, lower2, right2, up2, 10, 10, prevFrame.copy(), frame.copy(), "second line", millis, second_box_log_array)) # <<-- here
                    t3 = threading.Thread(name="box3_thread", target=control_box(left3, lower3, right3, up3, 10, 10, prevFrame.copy(), frame.copy(), "third line", millis, third_box_log_array)) # <<-- here
                    t4 = threading.Thread(name="box4_thread", target=control_box(left4, lower4, right4, up4, 10, 10, prevFrame.copy(), frame.copy(), "four line", millis, four_box_log_array)) # <<-- here
                    
                    t1.start()
                    t2.start()
                    t3.start()
                    t4.start()
                    #matcher.checkMotionInFeturesBox()
'''
                    control_box(left1, lower1, right1, up1, 10, 10, prevFrame.copy(), frame.copy(), "first line", millis, first_box_log_array)
                    control_box(left2, lower2, right2, up2, 10, 10, prevFrame.copy(), frame.copy(), "second line", millis, second_box_log_array)
                    #control_box(left3, lower3, right3, up3, 10, 10, prevFrame.copy(), frame.copy(), "first line under", millis, third_box_log_array)
                    #control_box(left4, lower4, right4, up4, 10, 10, prevFrame.copy(), frame.copy(), "second line under", millis, four_box_log_array)
                    frame = cv.rectangle(frame, (left1, lower1), (right1, up1), (255,0,255), 3)
                    frame = cv.rectangle(frame, (left2, lower2), (right2, up2), (255,0,255), 3)
                    frame = cv.rectangle(frame, (left3, lower3), (right3, up3), (255,0,255), 3)
                    frame = cv.rectangle(frame, (left4, lower4), (right4, up4), (255,0,255), 3)
                    
                    self.currentFrame = frame
                    #cv.line(frame, (int(sumLen), 0), (int(sumLen), 1000), (255, 0, 255), 2)
                    cv.imshow("mat", self.currentFrame)
                    #cv.waitKey(0)
                else:
                    init = False
                #print(sumLen)
                prevFrame = frameInit.copy()
                #frameWrap.showFrame()

                if cv.waitKey(1) & 0xFF == ord('q'):
                    break
            
            elif ret == True:
                continue
            else:
                break
        cap.release()
        cv.destroyAllWindows()


# УКАЗАТъ ПРАВИЛЬНЫЙ ПУТЬ ДО ВИДЕО
Application("18391-video.mp4").run()