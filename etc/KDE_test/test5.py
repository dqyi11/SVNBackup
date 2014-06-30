'''
Created on Jan 21, 2014

@author: daqing_yi
'''

if __name__ == '__main__':
    
    import numpy as np
    from sklearn import gaussian_process
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    
    def f(x1, x2):
        return - ( 2 * x1**2 + 3 * x2**2 )
    X = np.zeros((5, 2));
    y = np.zeros((5, 1));

    for i in range(5):
        X[i,0] = i
        X[i,1] = i + 1    
        y[i,0] = f(X[i,0], X[i,1])
        
    
    gp = gaussian_process.GaussianProcess(theta0=1e-2, thetaL=1e-4, thetaU=1e-1)
    gp.fit(X, y) 
    
    gp.fit(X, y)
    
    X, Y = np.mgrid[-5:5:100j, -5:5:100j]
    positions = np.vstack([X.ravel(), Y.ravel()]).T
    print positions
    y_pred, sigma2_pred = gp.predict(positions, eval_MSE=True)
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(positions[:,0], positions[:,1], y_pred)
    plt.show()