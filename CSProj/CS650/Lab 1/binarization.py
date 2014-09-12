'''
Created on Sep 6, 2014

@author: daqing_yi
'''
import numpy as np

def otsu(hist, total):
    
    intensity_level_num = len(hist)
    
    sum = 0
    for i in range(intensity_level_num):
        sum += i * hist[i]
    #print sum
    
    intraclass_variances = np.zeros(intensity_level_num)
        
    background_weight = 0.0
    background_mean = 0.0
    background_sum = 0.0
    
    foreground_weight = 0.0
    foreground_mean = 0.0
    foreground_sum = 0.0
    
    max_intraclass_variance = 0.0
    
    threshold_low = 0.0
    threshold_high = 0.0
    
    for i in range(intensity_level_num):
        background_weight += hist[i]
        if background_weight == 0.0:
            continue
        foreground_weight = total - background_weight
        if foreground_weight == 0.0:
            break
        background_sum += i * float(hist[i])
        foreground_sum = sum - background_sum
        
        background_mean = float(background_sum) / background_weight
        foreground_mean = float(foreground_sum) / foreground_weight
        
                
        intraclass_variances[i] = float(background_weight) * float(foreground_weight) * np.power( background_mean - foreground_mean, 2  )
        
        #print "I:" + str(i) + " var:" + str(intraclass_variance)
        
        if intraclass_variances[i] >= max_intraclass_variance:
            threshold_low = i
            if intraclass_variances[i] > max_intraclass_variance:
                threshold_high = i
            max_intraclass_variance = intraclass_variances[i]
    
    print "LOW: " + str(threshold_low) + " HIGH: " + str(threshold_high)        
    return (threshold_low + threshold_high)/2.0, intraclass_variances
    
def binarize(img_data, threshold, on_value=1):
        
    binary_data = np.zeros(img_data.shape, np.int)
    for i in range(img_data.shape[0]):
        for j in range(img_data.shape[1]):
            #print str(img_data[i,j]) + " : " + str(threshold)
            if img_data[i,j] > threshold:
                binary_data[i,j] = on_value
                
    return binary_data




            
            
            
        
    
    