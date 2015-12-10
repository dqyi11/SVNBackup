'''
Created on Dec 9, 2015

@author: walter
'''

from Map import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    FILENAME = 'map01.jpg'
    map = Map( FILENAME )

    #print map.triangle_length
    
    viz_data = np.zeros( ( map.width, map.height ), np.int )

    #print map.pixmap

    '''
    ps = map.get_circle(30, 100, 60)
    for p in ps:
        viz_data[p[1], p[0]] = 255
    '''
    for x in range( map.width ):
        for y in range( map.height ):
            if False == map.is_in_obstacle( x, y ):
                print str(x) + "-" + str(y)
                viz_data[ x, y ] = map.dist_to_nearest_obstacle(x, y)
                #print viz_data[ x ,y ]
                       
                
    #viz_min = viz_data.min()
    #viz_max = viz_data.max()
    
    #viz_data = 255 * ( viz_data - viz_min ) / (viz_max - viz_min )
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.imshow( viz_data )
    plt.show()