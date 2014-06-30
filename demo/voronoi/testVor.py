from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw
import numpy as np
import random

width = 100
height = 100
numcell = 10

if __name__ == '__main__':
    
    image = Image.new("RGB", (width, height))
    pixel = image.load()
    imgx, imgy = image.size
    
    point_list = []
    for i in range(numcell):
        point = [random.randrange(imgx), random.randrange(imgy)]
        point_list.append(point)
    points = np.array(point_list)
    
    #points = np.array([[0, 0], [0, 1], [0, 2], [1, 0], [1, 1], [1, 2], [2, 0], [2, 1], [2, 2]])
    vor = Voronoi(points)

    #print vor.vertices
    #print vor.regions
    #print vor.ridge_vertices
    #print vor.ridge_points
    
    #voronoi_plot_2d(vor)
    #plt.show()
    
    vex = []
    
    for s in vor.vertices:
        print s
        x = int(s[0])
        y = int(s[1])
        print str(x) + " - " + str(y)
        if x > 1 and x < imgx and y > 1 and y < imgy:
            pixel[x,y] = (255,255, 255)
        
    image.save("Voronoi.png", "PNG")
    #image.show()
    
