'''
Created on Sep 17, 2014

@author: daqing_yi
'''
import numpy as np
from AccumulatorMgr import *

def houghCircle(bi_img, radius):
    img_width = bi_img.shape[0]
    img_height = bi_img.shape[1]
    hough_img = np.zeros(bi_img.shape, np.int)
    
    print bi_img.shape
    
    for i in range(img_width):
        for j in range(img_height):
            if bi_img[i,j] > 0:
                pixels = getCircleEdgePixels([i,j], radius)
                for pix in pixels:
                    pix_x = pix[0]
                    pix_y = pix[1]
                    if pix_x >= 0 and pix_x < img_width and pix_y >= 0 and pix_y < img_height:
                        hough_img[pix_x, pix_y] += 1
    return hough_img

def houghCircles(bi_img, radii):
    radiiLen = len(radii)
    radiiMax = np.max(radii)
    
    img_width = bi_img.shape[0]
    img_height = bi_img.shape[1]
    
    accumulators = AccumulatorMgr()
    
    for r in radii:
        for i in range(img_width):
            for j in range(img_height):
                if bi_img[i,j] > 0:
                    pixels = getCircleEdgePixels([i,j], r)
                    for pix in pixels:
                        pix_x, pix_y = pix[0], pix[1]
                        if pix_x >= -r and pix_x < img_width+r and pix_y >= -r and pix_y < img_height+r:
                            accumulators.vote(pix_x, pix_y, r, [i,j])
                                      
    return accumulators

def oneVoterPerVoteMethod(accumulators, img_data):
    pass

def recursiveWeightedVoteMethod(accumulators, img_data):
    pass

def weightedRevote(accumulators, img_data):
                    

def findByThreshold(img_data, threshold):
    
    results = []
    img_width = img_data.shape[0]
    img_height = img_data.shape[1]
    
    for i in range(img_width):
        for j in range(img_height):
            if img_data[i,j] > threshold:
                results.append([i,j])
    return results


def findLocalMax(hough_img):
    local_max = []
    
    hough_img_min = np.min(np.min(hough_img))
    hough_img_max = np.max(np.max(hough_img))
    img_width = hough_img.shape[0]
    img_height = hough_img.shape[1]
    
    print hough_img_min
    print hough_img_max
    
    threshold = 0.9 *(hough_img_max - hough_img_min) + hough_img_min
    
    for i in range(img_width):
        for j in range(img_height):
            if hough_img[i,j] > threshold:
                local_max.append([i,j])
    
    return local_max
        

def getCircleEdgePixels(center, radius):
    pixels = []
    for theta in range(0, 360, 2):
        theta_radius = theta*np.pi/180.0
        x = int(center[0] + radius * np.cos(theta_radius))
        y = int(center[1] + radius * np.sin(theta_radius))
        if [x,y] not in pixels:
            pixels.append([x, y])
        #else:
        #    print 'dup'
    return pixels


