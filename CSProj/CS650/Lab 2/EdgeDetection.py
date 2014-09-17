'''
Created on Sep 17, 2014

@author: daqing_yi
'''
import numpy as np

def sobel(img_data):
    
    img_width = img_data.shape[0]
    img_height = img_data.shape[1]
    
    sobel_filter_size = 3
    
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
            
    return gradient_magnitude, gradient_orientation

def laplacian(img_data):
            
    img_width = img_data.shape[0]
    img_height = img_data.shape[1]
    
    sobel_filter_size = 3
    
    laplacian_kernel = np.array([[0, 1, 0], [1, -4, 1], [0, 1, 0]])
    
    g_m = np.zeros(img_data.shape, np.float)
    
    for i in range(1, img_width-1):
        for j in range(1, img_height-1):
            
            img_data_seg = np.array(img_data[i-1:i+2, j-1:j+2])
            g_m[i, j] = np.sum( np.sum( img_data_seg * laplacian_kernel ) )
            
    return g_m            
            
            
            