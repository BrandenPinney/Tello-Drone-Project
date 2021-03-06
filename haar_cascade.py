'''Haar Cascade detection through OpenCV using the OpenCV documentation. By Branden Pinney and Shayne Duncan 2022.'''

import cv2 as cv
import os

CWD = os.getcwd()

def findTurbine(img):
    '''Take an input image and searches for the target object using an xml file. 
    Returns the inupt image with boundaries drawn around the detected object and the x and y values of the center of the target in the image
    as well as the area of the detection boundary.'''
    # Use Haar Cascades to detect objects using the built-in classifier tool
    cascade = cv.CascadeClassifier(CWD + "\outdoor_cascade.xml")
    # clahe = cv.createCLAHE(clipLimit=3., tileGridSize=(8,8))

    # lab = cv.cvtColor(img, cv.COLOR_BGR2LAB)  # convert from BGR to LAB color space
    # l, a, b = cv.split(lab)  # split on 3 different channels

    # l2 = clahe.apply(l)  # apply CLAHE to the L-channel

    # lab = cv.merge((l2,a,b))  # merge channels
    # img = cv.cvtColor(lab, cv.COLOR_LAB2BGR)  # convert from LAB to BGR
    # img = cv.normalize(img, None, alpha=-1.5*255, beta=1.5*255, norm_type=cv.NORM_MINMAX, dtype=cv.CV_8U)

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    turbines = cascade.detectMultiScale(gray, 1.2, 8)

    turbineListC = []
    turbineListArea = []

    for (x,y,w,h) in turbines:
        # draw a rectangle around the detected object
        # code for creating a rectangle to see dectection boundaries -- 
        cv.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        # determine the center of the detection boundaries and the area
        centerX = x + w // 2
        centerY = y + h // 2
        area = w * h
        turbineListC.append([centerX, centerY])
        turbineListArea.append(area)
    if len(turbineListArea) != 0: 
        # if there is items in the area list, find the maximum value and return
        i = turbineListArea.index(max(turbineListArea))
        return img, [turbineListC[i], turbineListArea[i], w]
    else:
        return img, [[0, 0], 0, 0]

if __name__ == "__main__":
    cap = cv.VideoCapture(0)
    while True:
        _, img = cap.read()
        findTurbine(img)
        img = cv.resize(img, None, fx=1, fy=1, interpolation=cv.INTER_AREA)
        cv.imshow("Output", img)
        cv.waitKey(1)