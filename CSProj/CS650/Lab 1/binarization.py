'''
Created on Sep 6, 2014

@author: daqing_yi
'''

def otsu(hist, total):
    
    intensity_level_num = len(hist)
    
    sum = 0
    for i in range(intensity_level_num):
        sum += i * hist[i]
        
    background_weight = 0
    background_mean = 0.0
    background_sum = 0
    
    foreground_weight = 0
    foreground_mean = 0.0
    foreground_sum = 0
    
    max_interclass_variance = 0.0
    
    threshold_low = 0
    threshold_high = 0
    
    for i in range(intensity_level_num):
        background_weight += hist[i]
        if background_weight == 0:
            continue
        foreground_weight = total - background_weight
        if foreground_weight == 0:
            break
        background_sum += i * hist[i]
        foreground_sum = sum - background_sum
        
        background_mean = float(background_sum) / background_weight
        foreground_mean = float(foreground_sum) / background_weight
        
        interclass_variance = (float(background_weight)/sum) * (float(foreground_weight) / sum) * ( background_mean - foreground_mean  )**2
        
        print "I:" + str(i) + " var:" + str(interclass_variance)
        
        if interclass_variance >= max_interclass_variance:
            threshold_low = i
            if interclass_variance > max_interclass_variance:
                threshold_high = i
            max_interclass_variance = interclass_variance
            
    return (threshold_low + threshold_high)/2.0
    

            
            
            
        
    
    