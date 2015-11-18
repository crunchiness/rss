#!/usr/bin/env python

'''
Simple "Square Detector" program.
Loads several images sequentially and tries to find squares in each image.
'''

# Python 2/3 compatibility
import sys
import numpy as np
import cv2
from matplotlib import pyplot as plt

#not used
def angle_cos(p0, p1, p2):
    d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
    return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )

#not used
def find_squares(img):
    img = cv2.GaussianBlur(img, (5, 5), 0)
    squares = []
    for gray in cv2.split(img):
        for thrs in xrange(0, 255, 26):
            if thrs == 0:
                bin = cv2.Canny(gray, 0, 50, apertureSize=5)
                bin = cv2.dilate(bin, None)
            else:
                retval, bin = cv2.threshold(gray, thrs, 255, cv2.THRESH_BINARY)
            bin, contours, hierarchy = cv2.findContours(bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                cnt_len = cv2.arcLength(cnt, True)
                cnt = cv2.approxPolyDP(cnt, 0.02*cnt_len, True)
                if len(cnt) == 4 and cv2.contourArea(cnt) > 1000 and cv2.isContourConvex(cnt):
                    cnt = cnt.reshape(-1, 2)
                    max_cos = np.max([angle_cos( cnt[i], cnt[(i+1) % 4], cnt[(i+2) % 4] ) for i in xrange(4)])
                    if max_cos < 0.1:
                        squares.append(cnt)
    return squares

#not used
def detect():
    from glob import glob
    for fn in glob('../../cube-rss/*.jpg'):
        img = cv2.imread(fn)
        squares = find_squares(img)
        cv2.drawContours( img, squares, -1, (0, 255, 0), 3 )
        cv2.imshow('squares', img)
        ch = 0xFF & cv2.waitKey()
        if ch == 27:
            break
    cv2.destroyAllWindows()
    
#used
if __name__ == '__main__':
    from glob import glob
    extendBy = 20
    for fn in glob('../../cube-rss/*.jpg'):
        img = cv2.imread(fn)
        SIZEX,SIZEY = img.shape[:2]
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray,150,250,apertureSize = 3)
        #cv2.imshow('squares', edges)
        #cv2.waitKey(0) 
        #lines = cv2.HoughLines(edges,1,np.pi/180,50)
        
        minLineLength = 0
        maxLineGap =50
        blur = cv2.bilateralFilter(edges,9,100,100)
        
        lines = cv2.HoughLinesP(blur,2,np.pi/45,100,minLineLength,maxLineGap) 
        if lines is None:
            print "bla bla"
            continue
        
        for l in lines:
            for x1,y1,x2,y2 in l:
                p = np.array([x1, y1])
                r = np.array([x2, y2])
                d = np.linalg.norm(r - p)
                top = r + float(extendBy)/float(d) * (r-p)
                bottom = p - float(extendBy)/float(d) * (r-p)
                x_low = bottom[0]
                y_low = bottom[1]
                x_high = top[0]
                y_high = top[1]
                cv2.line(img,(int(x_low),int(y_low)),(int(x_high),int(y_high)),(0,255,0),2)
                cv2.line (img,(x1,y1),(x2,y2),(255,0,0),2)
                
            cv2.imshow('x',img)
       
        cv2.waitKey(0)

        
        """for (rho,theta) in lines[0]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            
            cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)
        
        plt.subplot(121),plt.imshow(blur,cmap = 'gray')
        plt.title('Original Image'), plt.xticks([]), plt.yticks([])
        plt.subplot(122),plt.imshow(edges,cmap = 'gray')
        plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
        plt.show() """
        
