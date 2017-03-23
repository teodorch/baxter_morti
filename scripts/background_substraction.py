#!/usr/bin/env python
import numpy as np
import cv2
import sys
import os

# both MOG and MOG2 can be used, with different parameter values
#backgroundSubtractor = cv2.BackgroundSubtractorMOG(3, 4, 0.8);
#backgroundSubtractor = cv2.BackgroundSubtractorMOG()

#backgroundSubtractor = cv2.BackgroundSubtractorMOG2(history=500, varThreshold=500)
#backgroundSubtractor = cv2.BackgroundSubtractorMOG2()

# apply the algorithm for background images using learning rate > 0

def substract_background(backgroundSubtractor, name):
    cwd = os.getcwd()
    for i in range(0, 10):
        bgImageFile =cwd + "/ros_ws/src/baxter_morti/images/backgrounds/" + name + "/frame%04d.jpg" % i
        print "Opening background", bgImageFile
        bg = cv2.imread(bgImageFile)
        cv2.imshow("asd", bg)
        backgroundSubtractor.apply(bg, learningRate=0.1)

# apply the algorithm for detection image using learning rate 0
    image = cv2.imread(cwd + "/ros_ws/src/baxter_morti/images/data/" + name + ".jpg")
    fgmask = backgroundSubtractor.apply(image, learningRate=0)
    cv2.imshow("image", fgmask)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    blurred = cv2.GaussianBlur(fgmask, (3, 3), 0)
 
    edged = cv2.Canny(blurred, 100, 150)

    ret,thresh = cv2.threshold(fgmask, 127,255,0)

    contours, hierarchy = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    mask = np.zeros(image.shape[:2],np.uint8)
    bgdModel = np.zeros((1,65),np.float64)
    fgdModel = np.zeros((1,65),np.float64)

    cnt=contours[0]
    x,y,w,h = cv2.boundingRect(cnt)
    rect = x,y,w,h

    for cnt in contours:
        nx,ny,nw,nh = cv2.boundingRect(cnt)
        if nw > w and nh > h:
           x = nx
           y = ny
           w = nw
           h = nh
           rect = x,y,w,h
       

    cv2.grabCut(image, mask, rect, bgdModel, fgdModel, 5, cv2.GC_INIT_WITH_RECT)

    cropped = image[y:y+h, x:x+w]
    cv2.imwrite(name +'_cropped.jpg',cropped)


    cv2.waitKey(0)
    cv2.destroyAllWindows()

def main():
    backgroundSubtractor = cv2.BackgroundSubtractorMOG(history=100, nmixtures=5, backgroundRatio=0.7, noiseSigma=0)
    substract_background(backgroundSubtractor, "left_1")
    substract_background(backgroundSubtractor, "left_2")
    substract_background(backgroundSubtractor, "left_3")
    substract_background(backgroundSubtractor, "right")


if __name__ == '__main__':
    main()
