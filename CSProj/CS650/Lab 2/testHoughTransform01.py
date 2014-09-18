'''
Created on Sep 18, 2014

@author: daqing_yi
'''

from EdgeDetection import *
from HoughTransform import *
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt


#img_filename = '2D_White_Box.pgm'
img_filename = 'blocks.pgm'
#img_filename = 'simplecircles.ppm'
#img_filename = 'circles.ppm'
img = Image.open(img_filename).convert("L")
img = np.array(img)

