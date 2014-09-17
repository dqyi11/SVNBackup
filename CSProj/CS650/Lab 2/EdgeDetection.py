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
    
    g_x = np.zeros(img_data.shape, np.float)
    g_y = np.zeros(img_data.shape, np.float)
    g_m = np.zeros(img_data.shape, np.float)
    
    for i in range(1, img_width-1):
        for j in range(1, img_height-1):
            
            img_data_seg = np.array(img_data[i-1:i+2, j-1:j+2])
            g_x[i,j] = np.sum( np.sum( sobel_x_kernel * img_data_seg ) )
            g_y[i,j] = np.sum( np.sum( sobel_y_kernel * img_data_seg ) )
            g_m[i,j] = np.sqrt(g_x[i,j]**2 + g_y[i,j]**2)
            
    return g_m

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
            
            
            