'''
Created on Sep 17, 2014

@author: daqing_yi
'''
import numpy as np
import copy

def gaussianFilter(img_data):
    
    img_gf = np.zeros(img_data.shape, np.int)
    img_width = img_data.shape[0]
    img_height = img_data.shape[1]
    gaussian_kernel = np.array([[0, 0, 1, 0, 0], [0, 1, 2, 1, 0], [1, 2, -16, 2, 1], [0, 1, 2, 1, 0], [0, 0, 1, 0, 0]])
    for i in range(2, img_width-2):
        for j in range(2, img_height-2):            
            img_data_seg = np.array(img_data[i-2:i+3, j-2:j+3])
            img_gf[i, j] = int( np.sum( np.sum( img_data_seg * gaussian_kernel ) ) )
                          
    return img_gf

def sobel(img_data):
    
    img_width = img_data.shape[0]
    img_height = img_data.shape[1]
        
    sobel_x_kernel = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]])
    sobel_y_kernel = np.array([[-1, -2, -1], [0, 0, 0], [1, 2, 1]])
    
    gradient_x = np.zeros(img_data.shape, np.float)
    gradient_y = np.zeros(img_data.shape, np.float)
    gradient_magnitude = np.zeros(img_data.shape, np.float)
    gradient_orientation = np.zeros(img_data.shape, np.float)
    
    for i in range(1, img_width-1):
        for j in range(1, img_height-1):            
            img_data_seg = np.array(img_data[i-1:i+2, j-1:j+2])
            gradient_x[i,j] = np.sum( np.sum( sobel_x_kernel * img_data_seg ) )
            gradient_y[i,j] = np.sum( np.sum( sobel_y_kernel * img_data_seg ) )
            gradient_magnitude[i,j] = np.sqrt(gradient_x[i,j]**2 + gradient_y[i,j]**2)
            gradient_orientation[i,j] = np.arctan2(gradient_y[i,j], gradient_x[i,j])
    
    gradient_magnitude_min = np.min(np.min(gradient_magnitude))
    gradient_magnitude_max = np.max(np.max(gradient_magnitude))
    gradient_magnitude = 255*(gradient_magnitude-gradient_magnitude_min)/(gradient_magnitude_max-gradient_magnitude_min)   
    return gradient_magnitude, gradient_orientation

def laplacian(img_data):
            
    img_width = img_data.shape[0]
    img_height = img_data.shape[1]
     
    laplacian_kernel = np.array([[0, 1, 0], [1, -4, 1], [0, 1, 0]])    
    hessian = np.zeros(img_data.shape, np.float)
    
    for i in range(1, img_width-1):
        for j in range(1, img_height-1):            
            img_data_seg = np.array(img_data[i-1:i+2, j-1:j+2])
            hessian[i, j] = np.sum( np.sum( img_data_seg * laplacian_kernel ) )    
    return hessian

def nonmaximalSuppresion(gradient_magnitude, gradient_orientation):
    
    gm_sup = np.zeros(gradient_magnitude.shape, gradient_magnitude.dtype)
    gm_width = gradient_magnitude.shape[0]
    gm_height = gradient_magnitude.shape[1]
    
    for i in range(1, gm_width-1):
        for j in range(1, gm_height-1):
            #orientation = gradient_orientation[i,j]
            
            degree = (180/np.pi)*gradient_orientation[i,j]
            
            if (degree<=22.5 and degree>-22.5) or degree<=-157.5 or degree>157.5:
                delta_i = 0
                delta_j = 1
            elif (degree>-67.5 and degree <= -22.5) or (degree>112.5 and degree<=157.5):
                delta_i = 1
                delta_j = -1
            elif (degree>-112.5 and degree<=-67.5) or (degree>67.5 and degree<=112.5):
                delta_i = 1
                delta_j = 0
            elif (degree>-157.5 and degree<=-112.5) or (degree>67.5 and degree<=22.5):
                delta_i = 1
                delta_j = 1
            
            if gradient_magnitude[i,j]>=gradient_magnitude[i-delta_i, j-delta_j] and gradient_magnitude[i,j]>=gradient_magnitude[i+delta_i, j+delta_j]:
                gm_sup[i,j] = gradient_magnitude[i,j]
    
    return gm_sup

def hysteresisThreshold(img_data, threshold_lo, threshold_hi):
    img_threshold = np.zeros(img_data.shape, np.float)
    img_width = img_data.shape[0]
    img_height = img_data.shape[1]
    
    for i in range(1,img_width-1):
        for j in range(1,img_height-1):
            if img_data[i,j] > threshold_hi:
                img_threshold[i,j] = 255
            elif img_data[i,j] > threshold_lo and img_data[i,j] <= threshold_hi:
                if img_data[i+1,j] > threshold_hi or img_data[i+1,j+1] > threshold_hi or img_data[i-1,j] > threshold_hi or img_data[i-1,j-1] > threshold_hi or img_data[i,j+1] > threshold_hi or img_data[i,j-1] > threshold_hi:
                    img_threshold[i,j] = 255
                    
    return img_threshold     

def canny(img_data, threshold_lo, threshold_hi):
    #calculate gradient
    img_gm, img_go = sobel(img_data)
    #nonmaxial suppression
    img_gm_proc = nonmaximalSuppresion(img_gm, img_go)
    #img_gm_proc = copy.deepcopy(img_gm)
    #hysteresis threshold
    img_can = hysteresisThreshold(img_gm_proc, threshold_lo, threshold_hi)
    #img_can = copy.deepcopy(img_gm_proc)
    return img_can

def MarrHildreth(img_data, threshold):
    
    img_width = img_data.shape[0]
    img_height = img_data.shape[1]
    
    img_gm, img_go = sobel(img_data)
    hessian = laplacian(img_data)
    
    img_mh = np.zeros(img_data.shape, np.float)

    for i in range(1, img_width-1):
        for j in range(1, img_height-1):
            
            if img_gm[i,j] >= threshold:
                degree = (180/np.pi)*img_go[i,j]
                
                if (degree<=22.5 and degree>-22.5) or degree<=-157.5 or degree>157.5:
                    delta_i = 0
                    delta_j = 1
                elif (degree>-67.5 and degree <= -22.5) or (degree>112.5 and degree<=157.5):
                    delta_i = 1
                    delta_j = -1
                elif (degree>-112.5 and degree<=-67.5) or (degree>67.5 and degree<=112.5):
                    delta_i = 1
                    delta_j = 0
                elif (degree>-157.5 and degree<=-112.5) or (degree>67.5 and degree<=22.5):
                    delta_i = 1
                    delta_j = 1
                    
                if (hessian[i+delta_i, j+delta_j] <= 0 and hessian[i-delta_i, j-delta_j]> 0):
                    img_mh[i,j] = 255
                elif (hessian[i+delta_i, j+delta_j] > 0 and hessian[i-delta_i, j-delta_j]<=0):
                    img_mh[i,j] = 255
                
                
    return img_mh
    

    
    
            
            
    
    
           
            
            
            