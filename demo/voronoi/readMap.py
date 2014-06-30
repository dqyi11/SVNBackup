from PIL import Image, ImageDraw
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':

    img = Image.open('map3.png').convert("L")
    imgx, imgy = img.size
    pix = img.load()
    
    nx = []
    ny = []
    obs_list = []
    
    for w in range(imgx):
        for h in range(imgy):
            if pix[w,h] < 10:
                obs_list.append([w,h])
                
    
    points = np.array(obs_list)
    vor = Voronoi(points)
    #voronoi_plot_2d(vor)
    #plt.show()
    
    img2 = Image.new("RGB",(imgx, imgy), "white")
    pix2 = img2.load()
    
    for o in obs_list:
        pix2[o[0],o[1]] = (0,0,0)
        
        
    for s in vor.vertices:
        #print s
        x = int(s[0])
        y = int(s[1])
        print str(x) + " - " + str(y)
        if x > 1 and x < imgx and y > 1 and y < imgy:
            pix2[x,y] = (0,0,255)
        
    img2.save("map3-voronoi.png", "PNG")
    





