'''
Created on Jan 20, 2014

@author: daqing_yi
'''

if __name__ == '__main__':
    
    import numpy as np
    import scipy.stats as stats
    from matplotlib.pyplot import imshow
    
    # Create some dummy data
    rvs = np.append(stats.norm.rvs(loc=2,scale=1,size=(2000,1)),
                    stats.norm.rvs(loc=0,scale=3,size=(2000,1)),
                    axis=1)
    
    kde = stats.kde.gaussian_kde(rvs.T)
    
    # Regular grid to evaluate kde upon
    x_flat = np.r_[rvs[:,0].min():rvs[:,0].max():128j]
    y_flat = np.r_[rvs[:,1].min():rvs[:,1].max():128j]
    x,y = np.meshgrid(x_flat,y_flat)
    grid_coords = np.append(x.reshape(-1,1),y.reshape(-1,1),axis=1)
    
    z = kde(grid_coords.T)
    z = z.reshape(128,128)
    
    imshow(z,aspect=x_flat.ptp()/y_flat.ptp())
        
        