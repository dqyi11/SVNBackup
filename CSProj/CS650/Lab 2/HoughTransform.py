'''
Created on Sep 17, 2014

@author: daqing_yi
'''
import numpy as np
from VoteGame import *
import scipy.ndimage as ndimage
import scipy.ndimage.filters as filters


def houghCircle(bi_img, radii):
    radiiLen = len(radii)
    radiiMax = np.max(radii)
    
    img_width = bi_img.shape[0]
    img_height = bi_img.shape[1]
    
    #vote_game = VoteGame(img_width+radiiMax, img_height+radiiMax, radii)
    img_hough = np.zeros((img_width+2*radiiMax, img_height+2*radiiMax,radiiLen), np.float)
    
    for ri in range(len(radii)):
        r = radii[ri]
        for i in range(img_width):
            for j in range(img_height):
                if bi_img[i,j] > 0:
                    pixels = getCircleEdgePixels([i,j], r)
                    for pix in pixels:
                        pix_x, pix_y = pix[0], pix[1]
                        if pix_x >= -r and pix_x < img_width+r and pix_y >= -r and pix_y < img_height+r:
                            img_hough[pix_x + r, pix_y + r, ri] += 1
                            
    img_hough_max = np.max(img_hough.flatten())
    img_hough_min = np.min(img_hough.flatten())
    
    img_hough = (img_hough - img_hough_min) / (img_hough_max - img_hough_min)
                                      
    return img_hough

def oneVoterPerVoteMethod(vote_game, img_data):
    pass

def recursiveWeightedVoteMethod(vote_game, img_data):
    pass

def weightedRevote(vote_game):
                
    for voter in vote_game.voterMgr.voters:
        weightSum = voter.getWeightedVoteSum()
        for v in voter.votes:
            v.weight /= weightSum
            
    return vote_game
        

def findByThreshold(img_data, threshold):
    
    results = []
    img_width = img_data.shape[0]
    img_height = img_data.shape[1]
    
    for i in range(img_width):
        for j in range(img_height):
            if img_data[i,j] > threshold:
                results.append([i,j])
    return results


def findLocalMax(data, threshold, neighborhood_size = 5):
    
    detected_peaks = []
    data_max = filters.maximum_filter(data, neighborhood_size)
    maxima = (data == data_max)
    data_min = filters.minimum_filter(data, neighborhood_size)
    diff = ((data_max - data_min) > threshold)
    maxima[diff == 0] = 0
    
    labeled, num_objects = ndimage.label(maxima)
    slices = ndimage.find_objects(labeled)
    x, y = [], []
    for dy,dx in slices:
        x_center = (dx.start + dx.stop - 1)/2
        y_center = (dy.start + dy.stop - 1)/2    
        detected_peaks.append([x_center, y_center])

    return detected_peaks
        

def getCircleEdgePixels(center, radius):
    pixels = []
    for theta in range(0, 360, 2):
        theta_radius = theta*np.pi/180.0
        x = int(center[0] + radius * np.cos(theta_radius))
        y = int(center[1] + radius * np.sin(theta_radius))
        if [x,y] not in pixels:
            pixels.append([x, y])

    return pixels


