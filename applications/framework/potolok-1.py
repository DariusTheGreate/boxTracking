import cv2
import numpy as np


cap = cv2.VideoCapture('1.mp4')
objectDetector = cv2.createBackgroundSubtractorMOG2(history = 100,varThreshold =5)

while cap.isOpened():
    ret, frame = cap.read()
    mask = objectDetector.apply(frame)
    if not ret:
        break
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    blur = cv2.medianBlur(gray, 5)
    sharpen_kernel = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
    sharpen = cv2.filter2D(blur, -1, sharpen_kernel)

    thresh = cv2.threshold(sharpen, 252, 255, cv2.THRESH_BINARY)[1]
    #cv2.imshow('thresh',thresh)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 200])
    higher_white = np.array([255, 255, 255])
    # getting the range of blue color in frame
    white_range = cv2.inRange(hsv, lower_white, higher_white)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    close = cv2.morphologyEx(white_range, cv2.MORPH_CLOSE, kernel, iterations=5)
    #cv2.imshow('close',close)

    cnts = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    best_contour = None
    max_square = 0
    for contour in cnts:
        if max_square < cv2.contourArea(contour):
            max_square = cv2.contourArea(contour)
            best_contour = contour
    approx = cv2.approxPolyDP(best_contour,0.001*cv2.arcLength(contour,True),True)

    x,y,w,h = cv2.boundingRect(approx)
    if 0.55<w/h<1.45:
        cv2.drawContours(frame,[approx],0,(0,255,0),5)
        print('cube found')
    else:
        print('not found')

   # cv2.drawContours(frame,cnts,-1,(0,255,0),3)
    cv2.imshow('vid', frame)
    #cv2.imshow('x',thresh)
    #cv2.imshow('a',close)
    if cv2.waitKey(1)==25:
        break

cap.release()
cv2.destroyAllWindows()

'''
image = cv2.imread('1.jpg')
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blur = cv2.medianBlur(gray, 5)
sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
sharpen = cv2.filter2D(blur, -1, sharpen_kernel)

thresh = cv2.threshold(sharpen,160,255, cv2.THRESH_BINARY)[1]
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
close = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

cnts = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnts = cnts[0] if len(cnts) == 2 else cnts[1]

min_area = 100
max_area = 1500
image_number = 0
best_contour = None
max_square = 0
for contour in cnts:
    if max_square< cv2.contourArea(contour):
        max_square=cv2.contourArea(contour)
        best_contour=contour
cv2.drawContours(image,[best_contour],-1,(0,255,0),3)
#cv2.imshow('sharpen', sharpen)
#cv2.imshow('close', close)
#cv2.imshow('thresh', thresh)
cv2.imshow('image', image)
cv2.waitKey()'''

