'''
Created on Sep 17, 2014

@author: daqing_yi
'''

import cv2
import numpy as np
import matplotlib.pyplot as plt

img_filename = '2D_White_Box.pgm'
img = cv2.imread(img_filename, 0)

img_sobel_x = cv2.Sobel(img, cv2.CV_64F, 1, 0, ksize=3)
img_sobel_y = cv2.Sobel(img, cv2.CV_64F, 0, 1, ksize=3)

img_sobel = np.zeros(img.shape, np.float)
for i in range(img.shape[0]):
    for j in range(img.shape[1]):
        img_sobel[i,j] = np.sqrt((img_sobel_x[i,j])**2 + (img_sobel_y[i,j])**2)
        
img_laplacian = cv2.Laplacian(img,cv2.CV_64F)

plt.subplot(1,3,1),plt.imshow(img,cmap = 'gray')
plt.title('Original'), plt.xticks([]), plt.yticks([])
plt.subplot(1,3,2),plt.imshow(img_sobel,cmap = 'gray')
plt.title('Sobel'), plt.xticks([]), plt.yticks([])
plt.subplot(1,3,3),plt.imshow(img_laplacian,cmap = 'gray')
plt.title('Laplacian'), plt.xticks([]), plt.yticks([])

plt.show()