import cv2
from matplotlib import pyplot as plt

class ObstacleFinder(object):
    
    def __init__(self):
        self.filename = None

    def loadFile(self, filename):
        self.filename = filename
        self.img = cv2.imread(filename, 0)
        self.edges = cv2.Canny(self.img, 100, 200)
        
    def showEdges(self):
        plt.subplot(111)
        plt.imshow(self.edges, cmap='gray')
        plt.show()