from ObstacleFinder import *

if __name__ == '__main__':
    
    finder = ObstacleFinder()
    finder.loadFile('two_houses.png')
    finder.showEdges()
    print finder.edges